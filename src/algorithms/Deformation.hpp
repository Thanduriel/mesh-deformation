#pragma once

#define _USE_MATH_DEFINES
#include <pmp/SurfaceMesh.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <array>
#include <memory>

namespace algorithm {

	class Deformation
	{
	public:
		// Construct a deformation operator on the given mesh.
		Deformation(pmp::SurfaceMesh& mesh);
		~Deformation();

		// Set the regions for deformations and computes the operator.
		// @param supportVertices The vertices that should be adjusted to minimize some energy functional.
		// @param handleVertices The vertices which are manually moved.
		void set_regions(const std::vector<pmp::Vertex>& supportVertices, 
			const std::vector<pmp::Vertex>& handleVertices);

		// Clears the selected regions, making the operator invalid.
		// Does not change the mesh.
		void reset_regions();

		// Reverts the mesh to the state of the last set_regions() call,
		// canceling changes to the handle.
		void reset_handle();

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

		// Applies the given transformation to the handle vertices 
		// and updates the vertices in the support region.
		void translate(const pmp::Normal& translation);
		void scale(pmp::Scalar scale);
		void rotate(const pmp::Normal& axis, pmp::Scalar angle);

		// recalculates the origin of the scale operator
		void reset_scale_origin();

		// Smoothing for detail preservation.
		// Use implicit smoothing instead of the displacement from the regular operator.
		void use_implict_smoothing(bool use);
		bool is_using_implict_smoothing() const { return useImplicitSmoothing_; }
		// Should details be added to the vertices in the support region.
		void show_details(bool show);
		bool is_showing_details() const { return showDetails_; }
		// Store details in the local frame or search in the neighborhood for the shortest displacement.
		void set_frame_search_depth(int nRing);
		int get_frame_search_depth() const { return searchNearestRing_; }

		// Parameters for implict smoothing.
		void set_smoothing_strength(pmp::Scalar timeStep);
		pmp::Scalar get_smoothing_strength() const { return smoothingTimeStep_; }
		// Same as set_order() but used in the implicit smoothing.
		void set_smoothing_order(int _order);
		int get_smoothing_order() const { return smoothingOrder_; }
	private:
		// matrix types in use
		using MatScalar = double; // scalar type for equation system solving
		using SparseMatrix = Eigen::SparseMatrix<MatScalar>;
		using SparseMatrixR = Eigen::SparseMatrix<MatScalar, Eigen::RowMajor>;
		using DenseMatrix = Eigen::Matrix<MatScalar, Eigen::Dynamic, Eigen::Dynamic>;
		using DiagonalMatrix = Eigen::DiagonalMatrix<MatScalar, Eigen::Dynamic>;
		using Triplet = Eigen::Triplet<MatScalar>;


		// Updates the lowResPositions of the support vertices with the current operator.
		// @param Apply changes to the actual points. Also adds details if showDetails_ == true.
		void update_support_region(bool apply = true);
		void compute_laplace();
		void compute_higher_order();
		// Recompute the operator with the current params and apply resulting changes to the mesh.
		void update_operator();

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
		// initial points in detailVectors_.
		void store_details();
		// Performs smoothing to the initial support region and updates encoded details.
		void compute_details();
		// Applies implicit smoothing to the support region storing the results in lowResPositions_.
		void implicit_smoothing(pmp::Scalar timeStep);
		// Computes a local frame based on the vertex normal and one edge.
		// @return The normalized orthogonal basis vectors.
		using Basis = std::tuple<pmp::Normal, pmp::Normal, pmp::Normal>;
		Basis local_frame(pmp::Vertex v) const;

		// Functor that wraps an Eigen solver to allow for different implementations
		// in the background.
		class BasicSolver
		{
		public:
			virtual ~BasicSolver() {}

			// Solve Ax = b. 
			// @param what Additional context info that is displayed if the solver fails.
			virtual DenseMatrix solve(const DenseMatrix& b, const char* what) = 0;
		};
		
		template<typename SolverT>
		class Solver : public BasicSolver
		{
		public:
			// Constructs the lhs.
			Solver(const SparseMatrix& A) : solver_(A) {}

			DenseMatrix solve(const DenseMatrix& b, const char* what) override
			{
				const DenseMatrix x = solver_.solve(b);
				if (solver_.info() != Eigen::Success)
					std::cerr << "Deformation: Could not solve linear system for " << what << ".\n";

				return x;
			}
		private:
			SolverT solver_;
		};

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
		pmp::VertexProperty<pmp::Point> initialPositions_; //< original positions at the time of set_regions()
		pmp::VertexProperty<pmp::Point> points_; //< standard point property "v:point" for convenience
		pmp::VertexProperty<Basis> localFrames_;
		pmp::VertexProperty<pmp::Vertex> localFrameIdx_;

		// Laplace operator
		SparseMatrixR laplacian_; //< symmetric laplacian
		SparseMatrix laplace1_; //< free vertices
		SparseMatrix laplace2_; //< fixed vertices
		DiagonalMatrix areaScale_; //< inverse Voronoi area
		DiagonalMatrix areaScale1Inv_; //< scale for only the free vertices
		DiagonalMatrix smoothnessScale_;
		std::unique_ptr<BasicSolver> solver_;
		int laplaceOrder_ = 2;
		bool useAreaScaling_ = true;

		pmp::Scalar smoothnessHandle_ = 2.0;
		pmp::Scalar smoothnessBoundary_ = 2.0;

		// precomputed basis functions
		DenseMatrix localHandle_;
		bool useBasisFunctions_ = true;
		DenseMatrix boundarySolution_;
		DenseMatrix handleBasis_;
		std::array<pmp::Point,4> affineFrame_;

		// Scale handling
		std::vector<pmp::Point> originScaleVertices_;
		std::vector<pmp::Point> originScaleFrame_;
		pmp::Point centerScale_;

		// smoothing related
		bool useImplicitSmoothing_ = false;
		bool showDetails_ = true;
		int searchNearestRing_ = 1;
		int smoothingOrder_ = 2;
		pmp::Scalar smoothingTimeStep_ = 0.001;
	};
}
