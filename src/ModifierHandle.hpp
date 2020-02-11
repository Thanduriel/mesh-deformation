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

class ModifierHandle {
public:
	ModifierHandle();
	~ModifierHandle();

	void draw(const mat4& projection_matrix, const mat4& view_matrix);

	void set_scale(const vec3 scale);
	void set_origin(const vec3& origin);
	void set_orientation(const Normal& forward, const Normal& up);

	vec3 compute_move_vector(const mat4& modelviewProjection, vec2 motion);
	void init_local_coordinate_system(mat4 modelview, vec3 normal);
	void set_local_axis(int axis);

	bool is_hit(const Ray& ray) const;
private:
	SurfaceColorMesh arrowMesh_;

	vec3 origin_;
	mat4 modelMatrix_;
	mat4 modelMatrixInverse_;
	mat4 scaleMatrix_;
	vec3 local_x_;
	vec3 local_y_;
	vec3 local_z_;

	vec3 currMoveAxis_;
};