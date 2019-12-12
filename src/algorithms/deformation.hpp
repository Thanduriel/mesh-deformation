#pragma once

#include <pmp/SurfaceMesh.h>

namespace algorithm {

	class Deformation
	{
	public:
		Deformation(pmp::SurfaceMesh& mesh);
		~Deformation();

		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices);

		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);
	private:
		void update_support_region();
		void compute_boundary_set();

		enum struct VertexType { None, Support, Handle, Boundary };

		pmp::SurfaceMesh& mesh_;
		std::vector<pmp::Vertex> supportVertices_;
		std::vector<pmp::Point> supportVertexPoints_;
		std::vector<pmp::Vertex> handleVertices_;
		std::vector<pmp::Vertex> boundaryVertices_;
		
		pmp::VertexProperty<VertexType> typeMarks_;
		pmp::VertexProperty<int> idx_;
	};
}