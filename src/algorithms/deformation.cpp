#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <chrono>
#include <Eigen/Dense>

namespace algorithm {

	using namespace pmp;

	using Triplet = Eigen::Triplet<double>;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh),
		typeMarks_(mesh_.add_vertex_property<VertexType>("v:typeMarks", VertexType::None)),
		idx_(mesh_.add_vertex_property<int>("v:newIdx", -1)),
		meshIdx_(mesh_.add_vertex_property<int>("v:meshIdx")),
		smoothness_(mesh_.add_vertex_property<pmp::Scalar>("v:smoothness", 2.0)),
		detailOffsets_(mesh_.add_vertex_property<pmp::Scalar>("v:detail", 0.0)),
		lowResPositions_(mesh_.add_vertex_property<pmp::Point>("v:lowResPosition")),
		laplacian_(mesh.n_vertices(), mesh.n_vertices()),
		areaScale_(mesh.n_vertices()),
		smoothnessScale_(mesh.n_vertices()),
		laplaceOrder_(3)
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
		mesh_.remove_vertex_property(detailOffsets_);
		mesh_.remove_vertex_property(lowResPositions_);
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

		int index = 0;
		for (Vertex v : supportVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Support; }
		for (Vertex v : handleVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Handle; }
		// an n-ring of size laplaceOrder_ is enough to fix the boundary
		compute_boundary_set(3);
		for (Vertex v : boundaryVertices_) { idx_[v] = index++; }

		smoothnessScale_.setIdentity();

		compute_laplace();
		compute_higher_order();

		implicit_smoothing(0.0001);
	}

	void Deformation::reset_regions()
	{
		supportVertices_.clear();
		handleVertices_.clear();
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

		for (Vertex v : handleVertices_) smoothness_[v] = smoothness;
		compute_higher_order();
		update_support_region();
	}

	void Deformation::set_smoothness_boundary(pmp::Scalar smoothness)
	{
		assert(is_set());

		for (Vertex v : boundaryVertices_) smoothness_[v] = smoothness;
		compute_higher_order();
		update_support_region();
	}

	void Deformation::translate(const pmp::Normal& translation)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");
		for (Vertex v : handleVertices_) points[v] += translation;

		for (int i = 0; i < 4; ++i) affineFrame_[i] += translation;

		update_support_region();
	}

	void Deformation::scale(Scalar scale)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		for (size_t i = 0; i < handleVertices_.size(); i++)
		{
			auto v = handleVertices_[i];
			points[v] = centerScale_ + (originScaleVertices_[i] - centerScale_) * scale;
		}
		for (size_t i = 0; i < affineFrame_.size(); i++)
		{
			auto fp = affineFrame_[i];
			affineFrame_[i] = centerScale_ + (originScaleFrame_[i] - centerScale_) * scale;
		}

		update_support_region();
	}

	void Deformation::rotate(const Normal& axis, Scalar angle)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		// find center point
		Point p(0.f);
		for (Vertex v : handleVertices_) p += points[v];
		p /= handleVertices_.size();

		mat4 rotMat = rotation_matrix(axis, angle);

		for (Vertex v : handleVertices_)
		{
			vec3 transVertex = points[v] - p;
			vec4 rotVertex = rotMat * vec4(transVertex, 0);
			points[v] = vec3(rotVertex[0], rotVertex[1], rotVertex[2]) + p;
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
		auto points = mesh_.get_vertex_property<Point>("v:point");
		originScaleVertices_.clear();
		originScaleFrame_.clear();

		// find center point
		centerScale_ = Point(0.0f);
		for (Vertex v : handleVertices_) centerScale_ += points[v];
		centerScale_ /= handleVertices_.size();

		for (Vertex v : handleVertices_)
			originScaleVertices_.push_back(points[v]);

		for (Point p : affineFrame_)
			originScaleFrame_.push_back(p);
	}

	void Deformation::update_support_region()
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		DenseMatrix X;
		if (useBasisFunctions_)
		{
			Eigen::Matrix<double, 4, 3> f;
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
				const Point p = points[v];
				x2(i, 0) = p[0];
				x2(i, 1) = p[1];
				x2(i, 2) = p[2];
			}

			const DenseMatrix B1 = -laplace2_ * x2;
			X = boundarySolution_ + solver_.solve(B1);
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
		assert(supportVertices_.size() && handleVertices_.size() && boundaryVertices_.size());

		// compute weights
		auto vweights = mesh_.add_vertex_property<Scalar>("v:area");
		for (Vertex v : mesh_.vertices())
		{
			vweights[v] = 0.5 / voronoi_area(mesh_, v);
			areaScale_.diagonal()[meshIdx_[v]] = vweights[v];
		}

		auto eweights = mesh_.add_edge_property<Scalar>("e:cotan");
		for (Edge e : mesh_.edges()) eweights[e] = std::max(0.0, cotan_weight(mesh_, e));

		const auto& meshIdx = meshIdx_;

		// construct Laplace operator matrix L and area weights M
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

		mesh_.remove_vertex_property(vweights);
		mesh_.remove_edge_property(eweights);
	}

	void Deformation::compute_higher_order()
	{
		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// use row major order to allow for quicker decomposition in L1 and L2
		const SparseMatrixR L = useAreaScaling_ ? areaScale_ * laplacian_ : laplacian_;
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
		SparseMatrix LDif = SparseMatrix(lOperator) - lOperator.transpose();
		//std::cout << "L: " << LDif.norm() << std::endl;

		decompose_operator(lOperator, laplace1_, laplace2_);
		
		//std::cout << "L1: " << (laplace1_ - SparseMatrix(laplace1_.transpose())).norm() << std::endl;
		auto begin = std::chrono::high_resolution_clock::now();
		solver_.compute(laplace1_);
		auto end = std::chrono::high_resolution_clock::now();
		//	std::cout << "decomposition:" << std::chrono::duration<double>(end - begin).count() << "\n";

			// precomputed basis functions
		auto points = mesh_.get_vertex_property<Point>("v:point");

		useBasisFunctions_ = compute_affine_frame();
		if (useBasisFunctions_)
		{
			std::cout << "Using precomputed basis functions.\n";
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
			handleBasis_ = solver_.solve(B1);
			if (solver_.info() != Eigen::Success)
				std::cerr << "Deformation: Could not solve linear system for handle vertices.\n";
		}

		// boundary vertices are fixed so this part of the rhs can be computed now
		DenseMatrix x3 = DenseMatrix::Zero(numFixed, 3);
		for (Vertex v : boundaryVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points[v];
			x3(i, 0) = p[0];
			x3(i, 1) = p[1];
			x3(i, 2) = p[2];
		}

		const DenseMatrix B2 = -laplace2_ * x3;

		boundarySolution_ = solver_.solve(B2);
		if (solver_.info() != Eigen::Success)
			std::cerr << "Deformation: Could not solve linear system for boundary vertices.\n";
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
		auto points = mesh_.get_vertex_property<Point>("v:point");

		affineFrame_[0] = pmp::Point(0.0, 0.0, 0.0);
		affineFrame_[1] = pmp::Point(1.0, 0.0, 0.0);
		affineFrame_[2] = pmp::Point(0.0, 1.0, 0.0);
		affineFrame_[3] = pmp::Point(0.0, 0.0, 1.0);


		Eigen::Matrix4d f;
		f << 0.0, 0.0, 0.0, 1.0,
			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0;

		DenseMatrix h(handleVertices_.size(), 4);
		for (size_t i = 0; i < handleVertices_.size(); ++i)
		{
			const Point p = points[handleVertices_[i]];
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
		auto points = mesh_.get_vertex_property<Point>("v:point");

		// compute new normals of the low resolution mesh
		for (Vertex v : supportVertices_) points[v] = lowResPositions_[v];
		auto normals = mesh_.add_vertex_property<Normal>("v:normal");
		for (Vertex v : supportVertices_) 
			normals[v] = SurfaceNormals::compute_vertex_normal(mesh_, v);
		
		// apply offsets to restore details
		for (Vertex v : supportVertices_)
			points[v] = lowResPositions_[v] + normals[v] * detailOffsets_[v];

		mesh_.remove_vertex_property(normals);
	}

	void Deformation::implicit_smoothing(Scalar timeStep)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();
		
		DenseMatrix x1(numFree, 3);
		for(Vertex v : supportVertices_)
		{
			const int i = idx_[v];
			const Point p = points[v];
			x1(i, 0) = p[0];
			x1(i, 1) = p[1];
			x1(i, 2) = p[2];
		}
		
		DenseMatrix x2 = DenseMatrix::Zero(numFixed, 3);
		for (Vertex v : handleVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points[v];
			x2(i, 0) = p[0];
			x2(i, 1) = p[1];
			x2(i, 2) = p[2];
		}
		for (Vertex v : boundaryVertices_)
		{
			const int i = idx_[v] - numFree;
			const Point p = points[v];
			x2(i, 0) = p[0];
			x2(i, 1) = p[1];
			x2(i, 2) = p[2];
		}

		SparseMatrix L1;
		SparseMatrix L2;
		decompose_operator(areaScale_ * laplacian_, L1, L2);

		SparseMatrix I(laplace1_.rows(), laplace1_.cols());
		I.setIdentity();

		Eigen::SparseLU<SparseMatrix> solver(I - (timeStep * L1));
		DenseMatrix X = solver.solve(x1 + timeStep * L2 * x2);

		for (size_t i = 0; i < supportVertices_.size(); ++i)
		{
			const Vertex v = supportVertices_[i];
			lowResPositions_[v][0] = X(i, 0);
			lowResPositions_[v][1] = X(i, 1);
			lowResPositions_[v][2] = X(i, 2);
			detailOffsets_[v] = pmp::distance(lowResPositions_[v], points[v]);
		}
	}
}