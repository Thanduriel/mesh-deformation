#pragma once

#ifndef M_PI
#define M_PI 3.1415 // expected by pmp
#endif // !M_PI
#include "algorithms/intersection.hpp"
#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>
#include <memory>
#include <unordered_set>

namespace algorithm {
	class Deformation;
}

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


class ModifierHandle {
public:
	ModifierHandle();
	~ModifierHandle();

	void draw(const mat4& projection_matrix, const mat4& view_matrix);

	void set_scale(const vec3 scale);
	void set_origin(const vec3& origin, const vec3& normal);
	void set_translationMode();
	void set_rotationMode();
	void set_scaleMode();

	void set_mouseStartPos(vec2 pos);

	vec2 get_mouseStartPos();

	bool is_translationMode();
	bool is_rotationMode();
	bool is_scaleMode();

	vec3 get_local_x() const { return local_x_; }
	vec3 get_local_y() const { return local_y_; }
	vec3 get_local_z() const { return local_z_; }

	vec3 compute_move_vector(float scalar);

	vec3 compute_move_vector();
	vec3 compute_rotation_vector();
	vec3 origin();

	void init_local_coordinate_system(mat4 modelview, vec3 normal);

	bool is_hit(const Ray& ray);
private:
	void precompute_intersection_structure();
	void precompute_intersection_structure(SurfaceColorMesh& mesh);
	
	void precompute_modelViewMatrix();
	mat4 compute_modelViewMatrix(vec3 forward, mat4 scaleMatrix);

	bool is_hit(const Ray& ray, mat4 modelMatrixInverse,const SurfaceColorMesh& mesh) const;

	void set_Selection(SurfaceColorMesh& mesh);

	void remove_Selection(SurfaceColorMesh & mesh);

	void remove_Selection();

	SurfaceColorMesh arrowMesh_LocalX_;
	SurfaceColorMesh arrowMesh_LocalY_;
	SurfaceColorMesh arrowMesh_LocalZ_;
	SurfaceColorMesh torusMesh_RotationX_;
	SurfaceColorMesh torusMesh_RotationY_;
	SurfaceColorMesh torusMesh_RotationZ_;
	SurfaceColorMesh scaleMesh_ScaleX_;
	SurfaceColorMesh scaleMesh_ScaleY_;
	SurfaceColorMesh scaleMesh_ScaleZ_;

	vec3 origin_;
	mat4 modelMatrixX_;
	mat4 modelMatrixInverseX_;
	mat4 modelMatrixY_;
	mat4 modelMatrixInverseY_;
	mat4 modelMatrixZ_;
	mat4 modelMatrixInverseZ_;
	mat4 modelMatrixRotationX_;
	mat4 modelMatrixInverseRotationX_;
	mat4 modelMatrixRotationY_;
	mat4 modelMatrixInverseRotationY_;
	mat4 modelMatrixRotationZ_;
	mat4 modelMatrixInverseRotationZ_;
	mat4 modelMatrixScaleX_;
	mat4 modelMatrixScaleInverseX_;
	mat4 modelMatrixScaleY_;
	mat4 modelMatrixScaleInverseY_;
	mat4 modelMatrixScaleZ_;
	mat4 modelMatrixScaleInverseZ_;

	mat4 scaleMatrixArrow_;
	mat4 scaleMatrixTorus_;
	mat4 scaleMatrixScale_;

	vec3 local_x_;
	vec3 local_y_;
	vec3 local_z_;

	vec2 mouseStartPos_;
	
	std::unordered_set<SurfaceColorMesh*> selectionMeshes_;
	std::unordered_set<SurfaceColorMesh*> updateMeshes_;

	EMode mode_;
};