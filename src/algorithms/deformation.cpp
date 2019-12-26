#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <Eigen/Sparse>

namespace algorithm {

	using namespace pmp;

	using Triplet = Eigen::Triplet<double>;
	using DenseMatrix = Eigen::MatrixXd;
		//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh),
		typeMarks_(mesh_.add_vertex_property<VertexType>("v:typeMarks", VertexType::None)),
		idx_(mesh_.add_vertex_property<int>("v:newIdx", -1))
	{
	}

	Deformation::~Deformation()
	{
		mesh_.remove_vertex_property(typeMarks_);
		mesh_.remove_vertex_property(idx_);
	}

	void Deformation::set_regions(const std::vector<Vertex>& supportVertices, 
		const std::vector<Vertex>& handleVertices)
	{
		supportVertices_ = supportVertices;
		handleVertices_ = handleVertices;

		int index = 0;
		for (Vertex v : supportVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Support; }
		for (Vertex v : handleVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Handle; }
		compute_boundary_set(2); // k + 1
		for (Vertex v : boundaryVertices_) { idx_[v] = index++; typeMarks_[v] = VertexType::Boundary; }

		compute_laplace();
	}

	void Deformation::translate(const pmp::Normal& translation)
	{
		auto points = mesh_.get_vertex_property<Point>("v:point");
		for (Vertex v : handleVertices_) points[v] += translation;
		update_support_region();
	}

	void Deformation::scale(Scalar scale)
	{
	}

	void Deformation::rotate(const Normal& axis, Scalar angle)
	{
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
		Eigen::SimplicialLDLT<SparseMatrix> solver(laplace1_);
		const DenseMatrix X = solver.solve(B);
		if (solver.info() != Eigen::Success)
			std::cerr << "Deformation: Could not solve linear system\n";
		else 
		{
			// apply result
			for (size_t i = 0; i < numFree; ++i)
			{
				points[supportVertices_[i]][0] = X(i, 0);
				points[supportVertices_[i]][1] = X(i, 1);
				points[supportVertices_[i]][2] = X(i, 2);
			}
		}
	}

	void Deformation::compute_laplace()
	{
		assert(supportVertices_.size() && handleVertices_.size() && boundaryVertices_.size());
		auto points = mesh_.get_vertex_property<Point>("v:point");
		auto areas = mesh_.add_face_property<Scalar>("f:area");

		// compute weights
		auto vweights = mesh_.add_vertex_property<Scalar>("v:area");
		for (Vertex v : mesh_.vertices()) vweights[v] = 0.5 / voronoi_area(mesh_, v);

		auto eweights = mesh_.add_edge_property<Scalar>("e:cotan");
		for (Edge e : mesh_.edges()) eweights[e] = std::max(0.0, cotan_weight(mesh_, e));

		auto meshIdx = mesh_.add_vertex_property<int>("v:meshIdx");
		int index = 0;
		for (Vertex v : mesh_.vertices()) meshIdx[v] = index++;

		const int numFree = supportVertices_.size();
		const int numFixed = handleVertices_.size() + boundaryVertices_.size();

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
		using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		SparseMatrixR L(mesh_.n_vertices(), mesh_.n_vertices());
		L.setFromTriplets(tripletsL.begin(), tripletsL.end());
		SparseMatrix M(mesh_.n_vertices(), mesh_.n_vertices());
		M.setFromTriplets(tripletsArea.begin(), tripletsArea.end());
		areaScale_ = std::move(M);
		// need other solver if this is used!!!
//		L = M * L;

		// extract submatrix of marked regions and reorder acording to idx_
		std::vector<Triplet> tripletsL1;
		std::vector<Triplet> tripletsL2;
		for (std::size_t i = 0; i < numFree; ++i)
		{
			int freeCount = 0;
			int fixedCount = 0;
			for (SparseMatrixR::InnerIterator it(L, meshIdx[supportVertices_[i]]); it; ++it)
			{
				const Vertex v(it.col());
				const int id = idx_[v];
				if (typeMarks_[v] == VertexType::Support)
				{
					tripletsL1.emplace_back(i, id, it.value());
				}
				else if(typeMarks_[v] != VertexType::None)
					tripletsL2.emplace_back(i, id - numFree, it.value());
			}
		}

		SparseMatrix L1(numFree, numFree);
		L1.setFromTriplets(tripletsL1.begin(), tripletsL1.end());
		laplace1_ = std::move(L1);
		SparseMatrix L2(numFree, numFixed);
		L2.setFromTriplets(tripletsL2.begin(), tripletsL2.end());
		laplace2_ = std::move(L2);

		mesh_.remove_vertex_property(meshIdx);
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
		for (int j = 0; j < ringSize-1; ++j)
		{
			const size_t end = boundaryVertices_.size();
			for (size_t i = begin; i < end; ++i)
				markNeigbhours(boundaryVertices_[i]);
			begin = end;
		}
	}
}