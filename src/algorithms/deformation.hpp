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
		// @param supportVertices The vertices that should be adjusted to minimize some energy functional.
		// @param handleVertices The vertices which are manually moved.
		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices);
		void reset_regions();

		// Set the energy form to be minimized as order of the operator L^k.
		void set_order(int k);
		int get_order() const { return laplaceOrder_; }
		
		// Enable area scaling of the Laplace operator to improve results for bad triangulations.
		void set_area_scaling(bool active);
		bool get_area_scaling() const { return useAreaScaling_; }

		// Factor which allows to locally interpolate between the different operators.
		// Only values between [0,k-1] where k is the order of the operator have an effect.
		// Requires that the regions are set.
		void set_smoothness_handle(pmp::Scalar smoothness);
		void set_smoothness_boundary(pmp::Scalar smoothness);

		// The operator is ready for modifications to the handle vertices.
		bool is_set() const;

		// Applies the given transformation to the handle vertices and updates the vertices in the support region.
		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);

		//TODO Comment
		void reset_scale_origin();

		void set_smoothing_strength(pmp::Scalar timeStep);
		pmp::Scalar get_smoothing_strength() const { return smoothingTimeStep_; }
		void show_details(bool show);
		bool is_showing_details() const { return showDetails_; }
	private:
		// matrix types in use
		using MatScalar = double; // scalar type for equation system solving
		using SparseMatrix = Eigen::SparseMatrix<MatScalar>;
		using SparseMatrixR = Eigen::SparseMatrix<MatScalar, Eigen::RowMajor>;
		using DenseMatrix = Eigen::Matrix<MatScalar, Eigen::Dynamic, Eigen::Dynamic>;
		using DiagonalMatrix = Eigen::DiagonalMatrix<MatScalar, Eigen::Dynamic>;
		using Triplet = Eigen::Triplet<MatScalar>;


		// Updates the positions of the support vertices with the current operator.
		void update_support_region();
		void compute_laplace();
		void compute_higher_order();
		// Decomposes the matrix into parts associated with the free and fixed vertices.
		// @param l1 Output target for the free vertices.
		// @param l2 Output target for the fixed vertices.
		void decompose_operator(const SparseMatrixR& lOperator, SparseMatrix& l1, SparseMatrix& l2) const;
		void compute_boundary_set(int ringSize);
		// @return success
		bool compute_affine_frame();
		
		// Recomputes the current mesh based on the low resolution version and detail offsets.
		void update_details();
		// Takes the current representation in lowResPositions_ and stores differences to the
		// real points in detailVectors_.
		void store_details();
		// Applies implicit smoothing to the support region and stores the results in lowResPositions_.
		void implicit_smoothing(pmp::Scalar timeStep);
		// Computes a local frame based on the vertex normal and one edge.
		std::tuple<pmp::Normal, pmp::Normal, pmp::Normal> local_frame(pmp::Vertex v) const;

		// mesh and modifier regions
		pmp::SurfaceMesh& mesh_;
		std::vector<pmp::Vertex> supportVertices_;
		std::vector<pmp::Vertex> handleVertices_;
		std::vector<pmp::Vertex> boundaryVertices_;

		// additional vertex properties
		enum struct VertexType { None, Support, Handle, Boundary };
		pmp::VertexProperty<VertexType> typeMarks_;
		pmp::VertexProperty<int> idx_;		//< indices for support region
		pmp::VertexProperty<int> meshIdx_;	//< indices for all vertices
		pmp::VertexProperty<pmp::Scalar> smoothness_;
		pmp::VertexProperty<pmp::Normal> detailVectors_; //< details in a local frame of the high resolution mesh
		pmp::VertexProperty<pmp::Point> lowResPositions_; //< positions in the low resolution representation
		pmp::VertexProperty<pmp::Point> initialPositions_; //< original positions at the time of set_regions().
		pmp::VertexProperty<pmp::Point> points_; //< standard point property "v:point" for convenience.

		// Laplace operator
		SparseMatrixR laplacian_;
		SparseMatrix laplace1_; //< free vertices
		SparseMatrix laplace2_; //< fixed vertices
		DiagonalMatrix areaScale_;
		DiagonalMatrix areaScale1Inv_; //< scale for only the free vertices
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

		// smoothing related
		bool showDetails_ = true;
		int smoothingOrder_ = 1;
		pmp::Scalar smoothingTimeStep_ = 0.000001;
	};
}