#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <chrono>

namespace algorithm {

	using namespace pmp;

	using Triplet = Eigen::Triplet<double>;
	using DenseMatrix = Eigen::MatrixXd;
	//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh),
		typeMarks_(mesh_.add_vertex_property<VertexType>("v:typeMarks", VertexType::None)),
		idx_(mesh_.add_vertex_property<int>("v:newIdx", -1)),
		meshIdx_(mesh_.add_vertex_property<int>("v:meshIdx")),
		smoothness_(mesh_.add_vertex_property<pmp::Scalar>("v:smoothness", 2.0)),
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
		mesh_.remove_vertex_property(smoothness_);
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
		compute_boundary_set(3); // actually laplaceOrder_ is sufficient
		for (Vertex v : boundaryVertices_) { idx_[v] = index++; }

		smoothnessScale_.setIdentity();

		compute_laplace();
		compute_higher_order();
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

	void Deformation::translate(const pmp::Normal& translation)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");
		for (Vertex v : handleVertices_) points[v] += translation;
		update_support_region();
	}

	void Deformation::scale(Scalar scale)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		// find center point
		Point p(0.f);
		for (Vertex v : handleVertices_) p += points[v];
		p /= handleVertices_.size();

		for (Vertex v : handleVertices_) points[v] = p + (points[v] - p) * scale;

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
		update_support_region();
	}

	void Deformation::update_support_region()
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");

		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// build right side B
		DenseMatrix x2(numFixed, 3);
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

		const DenseMatrix B = /*areaScale_ **/ -laplace2_ * x2;
		
		auto begin = std::chrono::high_resolution_clock::now();
		const DenseMatrix X = solver_.solve(B);
		auto end = std::chrono::high_resolution_clock::now();
	//	std::cout << std::chrono::duration<double>(end - begin).count() << "\n";
		
		if (solver_.info() != Eigen::Success)
		{
			std::cerr << "Deformation: Could not solve linear system\n";
			throw 42;
		}

		// apply result
		for (size_t i = 0; i < numFree; ++i)
		{
			points[supportVertices_[i]][0] = X(i, 0);
			points[supportVertices_[i]][1] = X(i, 1);
			points[supportVertices_[i]][2] = X(i, 2);
		}
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
		std::vector<Triplet> tripletsArea;
		for (Vertex v : mesh_.vertices())
		{
			tripletsArea.emplace_back(meshIdx[v], meshIdx[v], vweights[v]);
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

		const SparseMatrixR L = useAreaScaling_ ? areaScale_ * laplacian_ : laplacian_;
		SparseMatrixR lOperator = laplacian_;
		for (int i = laplaceOrder_- 2; i >= 0; --i)
		{
			auto& diagonal = smoothnessScale_.diagonal();
			for (Vertex v : handleVertices_)
				diagonal[meshIdx_[v]] = std::clamp(smoothness_[v] - i, Scalar(0.0), Scalar(1.0));
			for (Vertex v : boundaryVertices_)
				diagonal[meshIdx_[v]] = std::clamp(smoothness_[v] - i, Scalar(0.0), Scalar(1.0));
			lOperator = lOperator * smoothnessScale_ * L;
		}
		SparseMatrix LDif = SparseMatrix(lOperator) - lOperator.transpose();
		std::cout << "L: " << LDif.norm() << std::endl;

		// extract submatrix of marked regions and reorder acording to idx_
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

		laplace1_.resize(numFree, numFree);
		laplace1_.setFromTriplets(tripletsL1.begin(), tripletsL1.end());
		laplace2_.resize(numFree, numFixed);
		laplace2_.setFromTriplets(tripletsL2.begin(), tripletsL2.end());
		std::cout << "L1: " << (laplace1_ - SparseMatrix(laplace1_.transpose())).norm() << std::endl;
		auto begin = std::chrono::high_resolution_clock::now();
		solver_.compute(laplace1_);
		auto end = std::chrono::high_resolution_clock::now();
	//	std::cout << "decomposition:" << std::chrono::duration<double>(end - begin).count() << "\n";
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
}