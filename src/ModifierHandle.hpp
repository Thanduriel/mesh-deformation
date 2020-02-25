#pragma once

#ifndef M_PI
#define M_PI 3.1415 // expected by pmp
#endif // !M_PI
#include "algorithms/intersection.hpp"
#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>
#include <memory>

namespace algorithm {
	class Deformation;
}

enum class EMode
{
	None,
	Translation_X,
	Translation_Y,
	Translation_Z
};


class ModifierHandle {
public:
	ModifierHandle();
	~ModifierHandle();

	void draw(const mat4& projection_matrix, const mat4& view_matrix);

	void set_scale(const vec3 scale);
	void set_origin(const vec3& origin, const vec3& normal);

	vec3 compute_move_vector(const mat4& modelviewProjection, vec2 motion);
	void init_local_coordinate_system(mat4 modelview, vec3 normal);

	bool is_hit(const Ray& ray);
private:
	void precompute_intersection_structure();
	void precompute_intersection_structure(SurfaceColorMesh& mesh);
	
	void precompute_modelViewMatrix();
	mat4 compute_modelViewMatrix(vec3 forward);

	bool is_hit(const Ray& ray, mat4 modelMatrixInverse,const SurfaceColorMesh& mesh) const;

	SurfaceColorMesh arrowMesh_LocalX_;
	SurfaceColorMesh arrowMesh_LocalY_;
	SurfaceColorMesh arrowMesh_LocalZ_;

	vec3 origin_;
	mat4 modelMatrixX_;
	mat4 modelMatrixInverseX_;
	mat4 modelMatrixY_;
	mat4 modelMatrixInverseY_;
	mat4 modelMatrixZ_;
	mat4 modelMatrixInverseZ_;
	mat4 scaleMatrix_;
	vec3 local_x_;
	vec3 local_y_;
	vec3 local_z_;

	EMode mode_;
};