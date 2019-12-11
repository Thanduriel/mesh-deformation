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

		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);
	private:
		void update_support_region();

		pmp::SurfaceMesh& mesh_;
		std::vector<pmp::Vertex> supportVertices_;
		std::vector<pmp::Vertex> handleVertices_;
		std::vector<pmp::Vertex> boundaryVertices_;
	};
}