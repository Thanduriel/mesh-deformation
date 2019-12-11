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
		auto idx = mesh_.add_vertex_property<int>("v:idx", -1);
		auto marked = mesh_.add_vertex_property<bool>("v:marked", false);
		int index = 0;
		for (Vertex v : supportVertices)	{ idx[v] = index++; marked[v] = true; }
		for (Vertex v : handleVertices)		{ idx[v] = index++; marked[v] = true; }
		for (Vertex v : boundaryVertices)	{ idx[v] = index++; marked[v] = true; }

		std::vector<Triplet> triplets;
		const std::size_t supSize = supportVertices.size();
		for (std::size_t i = 0; i < supSize; ++i)
		{
			const Vertex v = supportVertices[i];

			Scalar sumWeights = 0.0;
			for (auto h : mesh_.halfedges(v))
			{
				const Vertex vv = mesh_.to_vertex(h);
				// maybe: still sum up
				if (marked[vv]) continue;

				const Scalar weight = cotan_weight(mesh_, mesh_.edge(h));
				sumWeights += weight;
				triplets.emplace_back(i, idx[vv], weight);
			}
			// maybe: 1.0/a in every factor
			triplets.emplace_back(i, idx[v], 1.0 / (Scalar(2.0) * voronoi_area(mesh_, v) - sumWeights));

		}

		SparseMatrix mat(supportVertices.size(), 
			supportVertices.size() + handleVertices.size() + boundaryVertices.size());
		mat.setFromTriplets(triplets.begin(), triplets.end());
	}

	void Deformation::translate(const pmp::dvec3& translation)
	{
	}

	void Deformation::scale(double scale)
	{
	}

	void Deformation::rotate(const pmp::dvec3& axis, double angle)
	{
	}
}