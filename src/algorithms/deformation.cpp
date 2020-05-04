#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <chrono>
#include <Eigen/Dense>
#include <unordered_set>

namespace algorithm {

	using namespace pmp;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh),
		typeMarks_(mesh_.add_vertex_property<VertexType>("v:typeMarks", VertexType::None)),
		idx_(mesh_.add_vertex_property<int>("v:newIdx", -1)),
		meshIdx_(mesh_.add_vertex_property<int>("v:meshIdx")),
		smoothness_(mesh_.add_vertex_property<Scalar>("v:smoothness", 2.0)),
		detailVectors_(mesh_.add_vertex_property<pmp::Normal>("v:detailV")),
		lowResPositions_(mesh_.add_vertex_property<Point>("v:lowResPosition")),
		initialPositions_(mesh_.add_vertex_property<Point>("v:initPosition")),
		points_(mesh_.get_vertex_property<Point>("v:point")),
		localFrames_(mesh.add_vertex_property<Basis>("v:localFrame")),
		localFrameIdx_(mesh.add_vertex_property<Vertex>("v:localFrameIdx")),
		laplacian_(mesh.n_vertices(), mesh.n_vertices()),
		areaScale_(mesh.n_vertices()),
		smoothnessScale_(mesh.n_vertices())
	{
		int index = 0;
		for (Vertex v : mesh_.vertices()) meshIdx_[v] = index++;
	}

	Deformation::~Deformation()
	{
		mesh_.remove_vertex_property(typeMarks_);
		mesh_.remove_vertex_property(idx_);
		mesh_.remove_vertex_property(meshIdx_);
		mesh_.remove_vertex_property(smoothness_);
		mesh_.remove_vertex_property(detailVectors_);
		mesh_.remove_vertex_property(lowResPositions_);
		mesh_.remove_vertex_property(initialPositions_);
	}

	void Deformation::set_regions(const std::vector<Vertex>& supportVertices,
		const std::vector<Vertex>& handleVertices)
	{
		for (Vertex v : mesh_.vertices())
		{
			idx_[v] = -1;
			typeMarks_[v] = VertexType::None;
		}

		supportVertices_ = supportVertices;
		handleVertices_ = handleVertices;

		// distribute global and per region indices
		int index = 0;
		for (Vertex v : supportVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Support; }
		for (Vertex v : handleVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Handle; }
		// an n-ring of size laplaceOrder_ is enough to fix the boundary
		compute_boundary_set(3);
		for (Vertex v : boundaryVertices_) { idx_[v] = index++; }

		for (Vertex v : supportVertices_) initialPositions_[v] = points_[v];
		for (Vertex v : handleVertices_) initialPositions_[v] = points_[v];

		smoothnessScale_.setIdentity();

		compute_laplace();
		compute_higher_order();

		implicit_smoothing(smoothingTimeStep_);

		// update vertices now to not have them jump with the first modification
		update_support_region();
	}

	void Deformation::reset_regions()
	{
		supportVertices_.clear();
		handleVertices_.clear();
	}

	void Deformation::reset_handle()
	{
		assert(is_set());

		for (Vertex v : handleVertices_)
			points_[v] = initialPositions_[v];

		compute_affine_frame();
		reset_scale_origin();
		update_support_region();
		update_details();
		
	}

	void Deformation::set_order(int k)
	{
		assert(k > 0 && k <= 3);
		laplaceOrder_ = k;

		if (is_set())
		{
			compute_higher_order();
			update_support_region();
		}
	}

	void Deformation::set_area_scaling(bool active)
	{
		useAreaScaling_ = active;

		if (is_set())
		{
			compute_higher_order();
			update_support_region();
		}
	}

	void Deformation::set_smoothness_handle(pmp::Scalar smoothness)
	{
		assert(is_set());

		smoothnessHandle_ = smoothness;
		for (Vertex v : handleVertices_) smoothness_[v] = smoothness;
		compute_higher_order();
		update_support_region();
	}

	void Deformation::set_smoothness_boundary(pmp::Scalar smoothness)
	{
		assert(is_set());

		smoothnessBoundary_ = smoothness;
		for (Vertex v : boundaryVertices_) smoothness_[v] = smoothness;
		compute_higher_order();
		update_support_region();
	}

	void Deformation::translate(const pmp::Normal& translation)
	{
		for (Vertex v : handleVertices_) points_[v] += translation;

		for (int i = 0; i < 4; ++i) affineFrame_[i] += translation;

		update_support_region();
	}

	void Deformation::scale(Scalar scale)
	{
		for (size_t i = 0; i < handleVertices_.size(); i++)
		{
			auto v = handleVertices_[i];
			points_[v] = centerScale_ + (originScaleVertices_[i] - centerScale_) * scale;
		}
		for (size_t i = 0; i < affineFrame_.size(); i++)
		{
			affineFrame_[i] = centerScale_ + (originScaleFrame_[i] - centerScale_) * scale;
		}

		update_support_region();
	}

	void Deformation::rotate(const Normal& axis, Scalar angle)
	{
		Point p(0.f);
		for (Vertex v : handleVertices_) p += points_[v];
		p /= handleVertices_.size();

		mat4 rotMat = rotation_matrix(axis, angle);

		for (Vertex v : handleVertices_)
		{
			vec3 transVertex = points_[v] - p;
			vec4 rotVertex = rotMat * vec4(transVertex, 0);
			points_[v] = vec3(rotVertex[0], rotVertex[1], rotVertex[2]) + p;
		}
		for (Point& fp : affineFrame_)
		{
			vec3 transVertex = fp - p;
			vec4 rotVertex = rotMat * vec4(transVertex, 0);
			fp = vec3(rotVertex[0], rotVertex[1], rotVertex[2]) + p;
		}

		update_support_region();
	}

	void Deformation::reset_scale_origin()
	{
		originScaleVertices_.clear();
		originScaleFrame_.clear();

		// find center point
		centerScale_ = Point(0.0f);
		for (Vertex v : handleVertices_) centerScale_ += points_[v];
		centerScale_ /= handleVertices_.size();

		for (Vertex v : handleVertices_)
			originScaleVertices_.push_back(points_[v]);

		for (Point p : affineFrame_)
			originScaleFrame_.push_back(p);
	}

	void Deformation::set_smoothing_strength(pmp::Scalar timeStep)
	{
		smoothingTimeStep_ = timeStep;

		// apply new results
		if (is_set())
		{
			// remember current configuration to restore quickly
			auto tempPoints = mesh_.vertex_property<Point>("v:tmpPoint");
			for (Vertex v : handleVertices_)
			{
				tempPoints[v] = points_[v];
				points_[v] = initialPositions_[v];
			}
			for (Vertex v : supportVertices_)
			{
				tempPoints[v] = lowResPositions_[v];
				points_[v] = initialPositions_[v];
			}

			implicit_smoothing(smoothingTimeStep_);
			
			for (Vertex v : handleVertices_)
				points_[v] = tempPoints[v];
			for (Vertex v : supportVertices_)
				lowResPositions_[v] = tempPoints[v];

			update_details();
		}
	}

	void Deformation::set_smoothing_order(int _order)
	{
		smoothingOrder_ = _order;
		set_smoothing_strength(smoothingTimeStep_);
	}

	void Deformation::show_details(bool show)
	{
		showDetails_ = show;

		update_details();
	}

	void Deformation::set_frame_search_depth(int useNearest)
	{
		searchNearestRing_ = useNearest;

		set_smoothing_strength(smoothingTimeStep_);
	}

	void Deformation::update_support_region()
	{
		DenseMatrix X;
		if (useBasisFunctions_)
		{
			Eigen::Matrix<MatScalar, 4, 3> f;
			for (int i = 0; i < 4; ++i)
			{
				f(i, 0) = affineFrame_[i][0];
				f(i, 1) = affineFrame_[i][1];
				f(i, 2) = affineFrame_[i][2];
			}

			X = boundarySolution_ + handleBasis_ * f;
		}
		else
		{
			const std::size_t numFree = supportVertices_.size();
			const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

			DenseMatrix x2 = DenseMatrix::Zero(numFixed, 3);
			for (Vertex v : handleVertices_)
			{
				const int i = idx_[v] - numFree;
				const Point p = points_[v];
				x2(i, 0) = p[0];
				x2(i, 1) = p[1];
				x2(i, 2) = p[2];
			}

			const DenseMatrix B1 = -laplace2_ * x2;
			X = boundarySolution_ + solver_->solve(B1, "support vertices");
		}

		// apply result
		for (size_t i = 0; i < supportVertices_.size(); ++i)
		{
			lowResPositions_[supportVertices_[i]][0] = X(i, 0);
			lowResPositions_[supportVertices_[i]][1] = X(i, 1);
			lowResPositions_[supportVertices_[i]][2] = X(i, 2);
		}

		update_details();
	}

	void Deformation::compute_laplace()
	{
		assert(is_set());

		const size_t numFree = supportVertices_.size();
		areaScale1Inv_.resize(numFree);
		// compute weights
		for (Vertex v : mesh_.vertices())
		{
			const Scalar vw = 0.5 / voronoi_area(mesh_, v);
			areaScale_.diagonal()[meshIdx_[v]] = vw;

			// weights of free vertices can be moved to the right side to preserve symmetry
			if (typeMarks_[v] == VertexType::Support)
				areaScale1Inv_.diagonal()[idx_[v]] = 1.0 / vw;
		}

		auto eweights = mesh_.add_edge_property<Scalar>("e:cotan");
		for (Edge e : mesh_.edges()) eweights[e] = std::max(0.0, cotan_weight(mesh_, e));

		const auto& meshIdx = meshIdx_;

		// construct the full symmetric Laplace operator matrix L
		std::vector<Triplet> tripletsL;
		for (Vertex v : mesh_.vertices())
		{
			Scalar sumWeights = 0.0;
			for (auto h : mesh_.halfedges(v))
			{
				const Vertex vv = mesh_.to_vertex(h);
				const Scalar weight = eweights[mesh_.edge(h)];
				sumWeights += weight;
				tripletsL.emplace_back(meshIdx[v], meshIdx[vv], weight);
			}
			tripletsL.emplace_back(meshIdx[v], meshIdx[v], -sumWeights);
		}

		// use row major to allow for quicker extraction of L1 and L2
		laplacian_.setFromTriplets(tripletsL.begin(), tripletsL.end());

		mesh_.remove_edge_property(eweights);
	}

	void Deformation::compute_higher_order()
	{
		auto start = std::chrono::high_resolution_clock::now();

		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// use row major order to allow for quicker decomposition into L1 and L2
		const SparseMatrixR L = useAreaScaling_ ? areaScale_ * laplacian_ : laplacian_;
		// the left most L does not need to be scaled since ML = 0 <=> L = 0
		SparseMatrixR lOperator = laplacian_;
		for (int i = laplaceOrder_ - 2; i >= 0; --i)
		{
			auto& diagonal = smoothnessScale_.diagonal();
			for (Vertex v : handleVertices_)
				diagonal[meshIdx_[v]] = std::clamp(smoothness_[v] - i, Scalar(0.0), Scalar(1.0));
			for (Vertex v : boundaryVertices_)
				diagonal[meshIdx_[v]] = std::clamp(smoothness_[v] - i, Scalar(0.0), Scalar(1.0));
			lOperator = lOperator * smoothnessScale_ * L;
		}

		decompose_operator(lOperator, laplace1_, laplace2_);
		
		// smoothness scale can make the operator non-symmetric for order 3
		const bool symmetric = laplaceOrder_ < 3 
			|| (smoothnessHandle_ == 2.0 && smoothnessBoundary_ == 2.0);
		solver_ = symmetric ? std::unique_ptr<BasicSolver>(new Solver< Eigen::SimplicialLDLT<SparseMatrix> > (laplace1_))
			: std::unique_ptr<BasicSolver>(new Solver< Eigen::SparseLU<SparseMatrix> >(laplace1_));;

		// precomputed basis functions
		useBasisFunctions_ = compute_affine_frame();
		if (useBasisFunctions_)
		{
			useBasisFunctions_ = true;

			DenseMatrix x2 = DenseMatrix::Zero(numFixed, 4);
			for (Vertex v : handleVertices_)
			{
				const int i = idx_[v] - numFree;
				x2(i, 0) = localHandle_(i, 0);
				x2(i, 1) = localHandle_(i, 1);
				x2(i, 2) = localHandle_(i, 2);
				x2(i, 3) = localHandle_(i, 3);
			}

			const DenseMatrix B1 = -laplace2_ * x2;
			handleBasis_ = solver_->solve(B1, "handle vertices");
		}

		// boundary vertices are fixed so this part of the rhs can be computed now
		DenseMatrix x3 = DenseMatrix::Zero(numFixed, 3);
		for (Vertex v : boundaryVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points_[v];
			x3(i, 0) = p[0];
			x3(i, 1) = p[1];
			x3(i, 2) = p[2];
		}

		const DenseMatrix B2 = -laplace2_ * x3;

		boundarySolution_ = solver_->solve(B2, "boundary vertices");

		auto end = std::chrono::high_resolution_clock::now();
	//	std::cout << std::chrono::duration<float>(end - start).count() << std::endl;
	}

	void Deformation::decompose_operator(const SparseMatrixR& lOperator, SparseMatrix& l1, SparseMatrix& l2) const
	{
		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// decompose into lhs, rhs and reorder acording to idx_
		std::vector<Triplet> tripletsL1;
		std::vector<Triplet> tripletsL2;
		for (std::size_t i = 0; i < numFree; ++i)
		{
			for (SparseMatrixR::InnerIterator it(lOperator, meshIdx_[supportVertices_[i]]); it; ++it)
			{
				const Vertex v(it.col());
				const int id = idx_[v];
				if (typeMarks_[v] == VertexType::Support)
				{
					tripletsL1.emplace_back(i, id, it.value());
				}
				else if (typeMarks_[v] != VertexType::None)
					tripletsL2.emplace_back(i, id - numFree, it.value());
			}
		}

		l1.resize(numFree, numFree);
		l1.setFromTriplets(tripletsL1.begin(), tripletsL1.end());
		l2.resize(numFree, numFixed);
		l2.setFromTriplets(tripletsL2.begin(), tripletsL2.end());
	}

	void Deformation::compute_boundary_set(int ringSize)
	{
		boundaryVertices_.clear();

		auto markNeigbhours = [this](Vertex v)
		{
			for (auto h : mesh_.halfedges(v))
			{
				const Vertex vv = mesh_.to_vertex(h);
				if (typeMarks_[vv] == VertexType::None)
				{
					typeMarks_[vv] = VertexType::Boundary;
					boundaryVertices_.push_back(vv);
				}
			}
		};

		for (Vertex v : supportVertices_) markNeigbhours(v);

		size_t begin = 0;
		for (int j = 0; j < ringSize - 1; ++j)
		{
			const size_t end = boundaryVertices_.size();
			for (size_t i = begin; i < end; ++i)
				markNeigbhours(boundaryVertices_[i]);
			begin = end;
		}
	}

	bool Deformation::is_set() const
	{
		return supportVertices_.size() && handleVertices_.size() && boundaryVertices_.size();
	}

	bool Deformation::compute_affine_frame()
	{
		affineFrame_[0] = pmp::Point(0.0, 0.0, 0.0);
		affineFrame_[1] = pmp::Point(1.0, 0.0, 0.0);
		affineFrame_[2] = pmp::Point(0.0, 1.0, 0.0);
		affineFrame_[3] = pmp::Point(0.0, 0.0, 1.0);


		Eigen::Matrix<MatScalar, 4, 4> f;
		f << 0.0, 0.0, 0.0, 1.0,
			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0;

		DenseMatrix h(handleVertices_.size(), 4);
		for (size_t i = 0; i < handleVertices_.size(); ++i)
		{
			const Point p = points_[handleVertices_[i]];
			h(i, 0) = p[0];
			h(i, 1) = p[1];
			h(i, 2) = p[2];
			h(i, 3) = 1;
		}

		// solve system Q*f = h for Q
		Eigen::ColPivHouseholderQR<DenseMatrix> solver(f.transpose());
		const DenseMatrix Q = solver.solve(h.transpose()).transpose();
		// check that the solution is valid 
		//if ((Q * f - h).norm() > 0.00001) return false;

		localHandle_ = Q;
		return true;
	}

	void Deformation::update_details()
	{
		if (showDetails_)
		{
			// compute new normals of the low resolution mesh
			for (Vertex v : supportVertices_) points_[v] = lowResPositions_[v];
			auto displacements = mesh_.add_vertex_property<Normal>("v:displacement");
			for (Vertex v : supportVertices_)
			{
				const auto& [b1, b2, b3] = local_frame(localFrameIdx_[v]);
				displacements[v] = b1 * detailVectors_[v][0] + b2 * detailVectors_[v][1] + b3 * detailVectors_[v][2];
			}

			// apply offsets to restore details
			for (Vertex v : supportVertices_)
				points_[v] = lowResPositions_[localFrameIdx_[v]] + displacements[v];

			mesh_.remove_vertex_property(displacements);
		}
		else
		{
			for (Vertex v : supportVertices_)
				points_[v] = lowResPositions_[v];
		}
	}

	void Deformation::store_details()
	{
		auto start = std::chrono::high_resolution_clock::now();
		// store low res positions for normal computations
		for (Vertex v : supportVertices_) 
			points_[v] = lowResPositions_[v];

		for (Vertex v : supportVertices_)
			localFrames_[v] = local_frame(v);
		
		auto marks = mesh_.add_vertex_property<bool>("v:tempMark", false);

		for (Vertex v : supportVertices_)
		{
			Normal dMin = initialPositions_[v] - points_[v];
			float lenSqMin = sqrnorm(dMin);
			Vertex frameIdx = v;
			
			// find point in the new mesh which is closest to the old position in the n-ring
			if (searchNearestRing_)
			{
				// collect vertices in the n-ring
				std::vector<Vertex> vertices;
				vertices.push_back(v);
				marks[v] = true;
				size_t begin = 0;

				for (int i = 0; i < searchNearestRing_; ++i)
				{
					const size_t end = vertices.size();
					for (size_t j = begin; j < end; ++j)
					{
						for (auto h : mesh_.halfedges(vertices[j]))
						{
							const Vertex vv = mesh_.to_vertex(h);
							if (typeMarks_[vv] == VertexType::Support && !marks[vv])
							{
								marks[vv] = true;
								vertices.push_back(vv);
							}
						}
					}
					begin = end;
				}

				// find nearest point
				for (Vertex vv : vertices)
				{
					marks[vv] = false;
					const Normal d = initialPositions_[v] - points_[vv];
					const float lenSq = sqrnorm(d);
					if (lenSq < lenSqMin)
					{
						dMin = d;
						lenSqMin = lenSq;
						frameIdx = vv;
					}
				}
			}
			// compute displacements in a local frame
			const auto& [n, b2, b3] = localFrames_[frameIdx];
			detailVectors_[v] = Normal(dot(dMin, n), dot(dMin, b2), dot(dMin, b3));
			localFrameIdx_[v] = frameIdx;
		}

		mesh_.remove_vertex_property(marks);

		// restore high resolution representation
		for (Vertex v : supportVertices_) 
			points_[v] = initialPositions_[v];
		auto end = std::chrono::high_resolution_clock::now();
	//	std::cout << std::chrono::duration<float>(end - start).count() << std::endl;
	}

	void Deformation::implicit_smoothing(Scalar timeStep)
	{
		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();
		
		DenseMatrix x1(numFree, 3);
		for(Vertex v : supportVertices_)
		{
			const int i = idx_[v];
			const Point p = points_[v];
			x1(i, 0) = p[0];
			x1(i, 1) = p[1];
			x1(i, 2) = p[2];
		}
		
		DenseMatrix x2 = DenseMatrix::Zero(numFixed, 3);
		for (Vertex v : handleVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points_[v];
			x2(i, 0) = p[0];
			x2(i, 1) = p[1];
			x2(i, 2) = p[2];
		}
		for (Vertex v : boundaryVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points_[v];
			x2(i, 0) = p[0];
			x2(i, 1) = p[1];
			x2(i, 2) = p[2];
		}

		SparseMatrix L1;
		SparseMatrix L2;
		SparseMatrixR lOperator;
		switch (smoothingOrder_)
		{
		case 3: lOperator = laplacian_ * areaScale_ * laplacian_ * areaScale_ * laplacian_;
			break;
		case 2: lOperator = laplacian_ * areaScale_ * laplacian_;
			break;
		case 1:
		default:
			lOperator = laplacian_;
		}
		decompose_operator(lOperator, L1, L2);

		SparseMatrix I(laplace1_.rows(), laplace1_.cols());
		I.setIdentity();

		// L1 is symmetric
		Eigen::SimplicialLDLT<SparseMatrix> solver(areaScale1Inv_ * I - (timeStep * L1));
		DenseMatrix X = solver.solve(areaScale1Inv_ * x1 + timeStep * L2 * x2);

		for (size_t i = 0; i < supportVertices_.size(); ++i)
		{
			const Vertex v = supportVertices_[i];
			lowResPositions_[v][0] = X(i, 0);
			lowResPositions_[v][1] = X(i, 1);
			lowResPositions_[v][2] = X(i, 2);
		}

		store_details();
	}

	std::tuple<Normal, Normal, Normal> Deformation::local_frame(Vertex v) const
	{
		const Normal n = SurfaceNormals::compute_vertex_normal(mesh_, v);
		const Halfedge h = *mesh_.halfedges(v).begin();
		const Vertex vv = mesh_.to_vertex(h);
		const Normal b2 = normalize(cross(n, points_[vv] - points_[v]));
		const Normal b3 = normalize(cross(n, b2));

		return { n,b2,b3 };
	}
}