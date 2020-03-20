#pragma once
#define _USE_MATH_DEFINES


#include <pmp/SurfaceMesh.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <array>

namespace algorithm {

	class Deformation
	{
	public:
		Deformation(pmp::SurfaceMesh& mesh);
		~Deformation();

		// Set the regions for deformations and computes the operator.
		// @param supportVertices The vertices that should be adjusted to minimize some enegy functional.
		// @param handleVertices The vertices which are manually moved.
		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices);
		void reset_regions();

		// Set the energy form to be minimized as order of the operator L^k.
		void set_order(int k);
		int get_order() const { return laplaceOrder_; }
		
		// Enable area scaling of the laplace operator to improve results for bad triangulations.
		void set_area_scaling(bool active);
		bool get_area_scaling() const { return useAreaScaling_; }

		// Factor which allows to localy interpolate between the different operators.
		// Only values between [0,k-1] where k is the order of the operator have an effect.
		// Requires that the regions are set.
		void set_smoothness_handle(pmp::Scalar smoothness);
		void set_smoothness_boundary(pmp::Scalar smoothness);

		// Operator is ready for modifications to the vertices.
		bool is_set() const;

		// Applies the given transformation the handle vertices and updates the vertices in the support region.
		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);

		//TODO Comment
		void reset_scale_origin();
	private:
		// matrix types in use
		using SparseMatrix = Eigen::SparseMatrix<double>;
		using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		using DenseMatrix = Eigen::MatrixXd;
		using DiagonalMatrix = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;

		// Updates the positions of the support vertices with the current operator.
		void update_support_region();
		void compute_laplace();
		void compute_higher_order();
		// decomposes the matrix into parts assosiated with the free and fixed vertices
		// @param l1 Output target for the free vertices.
		// @param l2 Output target for the fixed vertices.
		void decompose_operator(const SparseMatrixR& lOperator, SparseMatrix& l1, SparseMatrix& l2) const;
		void compute_boundary_set(int ringSize);
		// @return success
		bool compute_affine_frame();
		
		// Recomputes the current mesh based on the low resolution version and detail offsets.
		void update_details();
		// Applies implicit smoothing to the support region and stores the results in lowResPositions_
		void implicit_smoothing(pmp::Scalar timeStep);

		// mesh and modifier regions
		pmp::SurfaceMesh& mesh_;
		std::vector<pmp::Vertex> supportVertices_;
		std::vector<pmp::Vertex> handleVertices_;
		std::vector<pmp::Vertex> boundaryVertices_;
		
		// additional vertex properties
		enum struct VertexType { None, Support, Handle, Boundary };
		pmp::VertexProperty<VertexType> typeMarks_;
		pmp::VertexProperty<int> idx_;		//< indicies for support region
		pmp::VertexProperty<int> meshIdx_;	//< indicies for all vertices
		pmp::VertexProperty<pmp::Scalar> smoothness_;
		pmp::VertexProperty<pmp::Scalar> detailOffsets_; //< details along normal directions of the high resolution mesh
		pmp::VertexProperty<pmp::Point> lowResPositions_; //< positions in the low resolution representation

		using SparseMatrix = Eigen::SparseMatrix<double>;
		using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		using DenseMatrix = Eigen::MatrixXd;
		using DiagonalMatrix = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;
		// laplace operator
		SparseMatrixR laplacian_;
		SparseMatrix laplace1_; //< support region
		SparseMatrix laplace2_; //< boundary region 
		DiagonalMatrix areaScale_;
		DiagonalMatrix smoothnessScale_;
		Eigen::SparseLU<SparseMatrix> solver_; // SparseLU, SimplicialLLT, SimplicialLDLT
		int laplaceOrder_;
		bool useAreaScaling_ = false;

		// precomputed basis functions
		DenseMatrix localHandle_;
		bool useBasisFunctions_ = false;
		DenseMatrix boundarySolution_;
		DenseMatrix handleBasis_;
		std::array<pmp::Point,4> affineFrame_;

		// Scale handling
		std::vector<pmp::Point> originScaleVertices_;
		std::vector<pmp::Point> originScaleFrame_;
		pmp::Point centerScale_;
	};
}