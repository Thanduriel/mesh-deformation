#pragma once
#define _USE_MATH_DEFINES


#include <pmp/SurfaceMesh.h>
#include <Eigen/Sparse>

namespace algorithm {

	class Deformation
	{
	public:
		Deformation(pmp::SurfaceMesh& mesh);
		~Deformation();

		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices);
		void set_order(int k);
		int get_order() const { return laplaceOrder_; }
		void set_area_scaling(bool active);
		bool get_area_scaling() const { return useAreaScaling_; }
		void set_smoothness_handle(pmp::Scalar smoothness);

		// operator is ready for modifications to the vertices
		bool is_set() const;

		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);
	private:
		void update_support_region();
		void compute_laplace();
		void compute_higher_order();
		void compute_boundary_set(int ringSize);

		enum struct VertexType { None, Support, Handle, Boundary };

		pmp::SurfaceMesh& mesh_;
		std::vector<pmp::Vertex> supportVertices_;
		std::vector<pmp::Vertex> handleVertices_;
		std::vector<pmp::Vertex> boundaryVertices_;
		
		pmp::VertexProperty<VertexType> typeMarks_;
		pmp::VertexProperty<int> idx_;		//< indicies for support region
		pmp::VertexProperty<int> meshIdx_;	//< indicies for all vertices
		pmp::VertexProperty<pmp::Scalar> smoothness_;

		using SparseMatrix = Eigen::SparseMatrix<double>;
		using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		SparseMatrixR laplacian_;
		SparseMatrix laplace1_; // support region
		SparseMatrix laplace2_; // boundary region 
		Eigen::DiagonalMatrix<double, Eigen::Dynamic> areaScale_;
		Eigen::DiagonalMatrix<double, Eigen::Dynamic> smoothnessScale_;
		Eigen::SparseLU<SparseMatrix> solver_;
		int laplaceOrder_;
		bool useAreaScaling_ = false;
	};
}