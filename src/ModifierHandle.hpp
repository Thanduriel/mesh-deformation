#pragma once

#include "algorithms/Intersection.hpp"
#include "SurfaceColorMesh.hpp"
#include <pmp/visualization/MeshViewer.h>
#include <imgui.h>
#include <memory>
#include <unordered_set>

class ModifierHandle {
public:
	enum class EMode
	{
		None,
		Translation_X,
		Translation_Y,
		Translation_Z,
		Rotation_X,
		Rotation_Y,
		Rotation_Z,
		Scale_X,
		Scale_Y,
		Scale_Z
	};

	ModifierHandle();
	~ModifierHandle();

	void draw(const pmp::mat4& projection_matrix, const pmp::mat4& view_matrix);

	void set_scale(const pmp::vec3 scale);
	void set_origin(const pmp::vec3& origin, const pmp::vec3& normal);
	void set_translationMode();
	void set_rotationMode();
	void set_scaleMode();

	void set_mouseStartPos(pmp::vec2 pos);

	pmp::vec2 get_mouseStartPos();

	bool is_translationMode() const;
	bool is_rotationMode() const;
	bool is_scaleMode() const;

	pmp::vec3 get_active_translation_axis() const;
	std::vector<pmp::vec3> get_active_normal_axes();

	pmp::vec3 get_last_hit_point();

	pmp::vec3 compute_move_vector(float scalar);

	pmp::vec3 compute_move_vector();
	pmp::vec3 compute_rotation_vector();
	pmp::vec3 origin();

	void init_local_coordinate_system(pmp::mat4 modelview, pmp::vec3 normal);

	void set_origin(const pmp::vec3 & origin);

	void set_scale(float scale);

	bool is_hit(const Ray& ray);
private:
	void precompute_intersection_structure();
	void precompute_intersection_structure(SurfaceColorMesh& mesh);
	
	void precompute_modelViewMatrix();
	pmp::mat4 compute_modelViewMatrix(pmp::vec3 forward, pmp::mat4 scaleMatrix);

	std::optional<float> is_hit(const Ray& ray, pmp::mat4 modelMatrixInverse,const SurfaceColorMesh& mesh) const;

	void set_selection(SurfaceColorMesh& mesh);

	void remove_selection(SurfaceColorMesh & mesh);

	void remove_selection();

	SurfaceColorMesh arrowMesh_LocalX_;
	SurfaceColorMesh arrowMesh_LocalY_;
	SurfaceColorMesh arrowMesh_LocalZ_;
	SurfaceColorMesh torusMesh_RotationX_;
	SurfaceColorMesh torusMesh_RotationY_;
	SurfaceColorMesh torusMesh_RotationZ_;
	SurfaceColorMesh scaleMesh_ScaleX_;
	SurfaceColorMesh scaleMesh_ScaleY_;
	SurfaceColorMesh scaleMesh_ScaleZ_;

	pmp::vec3 origin_;
	pmp::mat4 modelMatrixX_;
	pmp::mat4 modelMatrixInverseX_;
	pmp::mat4 modelMatrixY_;
	pmp::mat4 modelMatrixInverseY_;
	pmp::mat4 modelMatrixZ_;
	pmp::mat4 modelMatrixInverseZ_;
	pmp::mat4 modelMatrixRotationX_;
	pmp::mat4 modelMatrixInverseRotationX_;
	pmp::mat4 modelMatrixRotationY_;
	pmp::mat4 modelMatrixInverseRotationY_;
	pmp::mat4 modelMatrixRotationZ_;
	pmp::mat4 modelMatrixInverseRotationZ_;
	pmp::mat4 modelMatrixScaleX_;
	pmp::mat4 modelMatrixScaleInverseX_;
	pmp::mat4 modelMatrixScaleY_;
	pmp::mat4 modelMatrixScaleInverseY_;
	pmp::mat4 modelMatrixScaleZ_;
	pmp::mat4 modelMatrixScaleInverseZ_;

	pmp::mat4 scaleMatrixArrow_;
	pmp::mat4 scaleMatrixTorus_;
	pmp::mat4 scaleMatrixScale_;

	pmp::vec3 local_x_;
	pmp::vec3 local_y_;
	pmp::vec3 local_z_;

	pmp::vec3 handleNormal;

	pmp::vec3 last_hit_point_;

	pmp::vec2 mouseStartPos_;
	
	std::unordered_set<SurfaceColorMesh*> selectionMeshes_;
	std::unordered_set<SurfaceColorMesh*> updateMeshes_;

	EMode mode_;
};