#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <Eigen/Sparse>

namespace algorithm {

	using namespace pmp;
	using namespace Eigen;

	using SparseMatrix = Eigen::SparseMatrix<double>;
	using Triplet = Eigen::Triplet<double>;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh),
		typeMarks_(mesh_.add_vertex_property<VertexType>("v:typeMarks", VertexType::None)),
		idx_(mesh_.add_vertex_property<int>("v:idx", -1))
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
		for (Vertex v : boundaryVertices_) { idx_[v] = index++; }

		// save old points to restore before setting up a new matrix
		auto points = mesh_.get_vertex_property<Point>("v:point");
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
		assert(supportVertices_.size() && handleVertices_.size() && boundaryVertices_.size());
		auto points = mesh_.get_vertex_property<Point>("v:point");
	
		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// construct Laplace operator matrix L
		std::vector<Triplet> tripletsL1;
		std::vector<Triplet> tripletsL2;
		for (std::size_t i = 0; i < numFree; ++i)
		{
			const Vertex v = supportVertices_[i];

			const Scalar a = 1.0;//Scalar(1.0) / (Scalar(2.0) * voronoi_area(mesh_, v));
			Scalar sumWeights = 0.0;
			for (auto h : mesh_.halfedges(v))
			{
				const Vertex vv = mesh_.to_vertex(h);
				const Scalar weight = cotan_weight(mesh_, mesh_.edge(h)) * a;
				sumWeights += weight;
				if (typeMarks_[vv] == VertexType::Support)
					tripletsL1.emplace_back(i, idx_[vv], weight);
				else
					tripletsL2.emplace_back(i, idx_[vv] - numFree, weight);
			}
			tripletsL1.emplace_back(i, idx_[v],  - sumWeights);
		}
		SparseMatrix L1(numFree, numFree);
		L1.setFromTriplets(tripletsL1.begin(), tripletsL1.end());
		SparseMatrix L2(numFree, numFixed);
		L2.setFromTriplets(tripletsL2.begin(), tripletsL2.end());

		// build right side B
		MatrixXd x2(numFixed, 3);
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
		const MatrixXd B = -L2 * x2;
		SimplicialLDLT<SparseMatrix> solver(L1);
		const Eigen::MatrixXd X = solver.solve(B);
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