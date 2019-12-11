#pragma once

#include <pmp/SurfaceMesh.h>

namespace algorithm {

	class Deformation
	{
	public:
		Deformation(pmp::SurfaceMesh& mesh);

		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices,
			const std::vector<pmp::Vertex>& boundaryVertices);

		void translate(const pmp::dvec3& translation);
		void scale(double scale);
		void rotate(const pmp::dvec3& axis, double angle);
	private:
		pmp::SurfaceMesh& mesh_;

		// todo:
	};
}