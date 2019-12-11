#include "deformation.hpp"
#include <pmp/algorithms/DifferentialGeometry.h>
#include <Eigen/Sparse>

namespace algorithm {

	using namespace pmp;
	using namespace Eigen;

	using SparseMatrix = Eigen::SparseMatrix<double>;
	using Triplet = Eigen::Triplet<double>;

	Deformation::Deformation(SurfaceMesh& mesh)
		: mesh_(mesh)
	{}

	void Deformation::set_regions(const std::vector<Vertex>& supportVertices, 
		const std::vector<Vertex>& handleVertices,
		const std::vector<Vertex>& boundaryVertices)
	{
		supportVertices_ = supportVertices;
		handleVertices_ = handleVertices;
		boundaryVertices_ = boundaryVertices;
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
		auto idx = mesh_.add_vertex_property<int>("v:idx", -1);

		enum struct Type { None, Support, Handle, Boundry };
		auto marked = mesh_.add_vertex_property<Type>("v:marked", Type::None);

		int index = 0;
		for (Vertex v : supportVertices_) { idx[v] = index++; marked[v] = Type::Support; }
		for (Vertex v : handleVertices_) { idx[v] = index++; marked[v] = Type::Handle; }
		for (Vertex v : boundaryVertices_) { idx[v] = index++; marked[v] = Type::Boundry; }
		const std::size_t numFree = supportVertices_.size();
		const std::size_t numFixed = handleVertices_.size() + boundaryVertices_.size();

		// construct Laplace operator matrix L
		std::vector<Triplet> tripletsL1;
		std::vector<Triplet> tripletsL2;
		for (std::size_t i = 0; i < numFree; ++i)
		{
			const Vertex v = supportVertices_[i];

			const Scalar a = Scalar(1.0) / (Scalar(2.0) * voronoi_area(mesh_, v));
			Scalar sumWeights = 0.0;
			for (auto h : mesh_.halfedges(v))
			{
				const Vertex vv = mesh_.to_vertex(h);
				// maybe: still sum up
				if (marked[vv] == Type::None) continue;

				const Scalar weight = cotan_weight(mesh_, mesh_.edge(h)) * a;
				sumWeights += weight;
				if (marked[vv] == Type::Support)
					tripletsL1.emplace_back(i, idx[vv], weight);
				else
					tripletsL2.emplace_back(i, idx[vv] - numFree, weight);
			}
			// maybe: 1.0/a in every factor
			tripletsL1.emplace_back(i, idx[v],  - sumWeights);
		}
		SparseMatrix L1(numFree, numFree);
		L1.setFromTriplets(tripletsL1.begin(), tripletsL1.end());
		SparseMatrix L2(numFree, numFixed);
		L2.setFromTriplets(tripletsL2.begin(), tripletsL2.end());

		// build right side B
		MatrixXd x2(numFixed, 3);
		for (Vertex v : handleVertices_)
		{
			const int i = idx[v] - numFree;
			const Point p = points[v];
			x2(i, 0) = p[0];
			x2(i, 1) = p[1];
			x2(i, 2) = p[2];
		}
		for (Vertex v : boundaryVertices_)
		{
			const int i = idx[v] - numFree;
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

		// clean-up
		// todo: move to destructor
		mesh_.remove_vertex_property(idx);
		mesh_.remove_vertex_property(marked);
	}
}