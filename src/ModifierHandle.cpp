#include "ModifierHandle.hpp"
#include "algorithms/intersection.hpp"
#include <iostream>

ModifierHandle::ModifierHandle()
	: scaleMatrix_(scaling_matrix(1.f))
{
	arrowMesh_LocalX_.read("../models/arrow.off");
	arrowMesh_LocalX_.update_opengl_buffers();

	arrowMesh_LocalY_.read("../models/arrow.off");
	arrowMesh_LocalY_.update_opengl_buffers();

	arrowMesh_LocalZ_.read("../models/arrow.off");
	arrowMesh_LocalZ_.update_opengl_buffers();

	precompute_modelViewMatrix();
	precompute_intersection_structure();
}

ModifierHandle::~ModifierHandle()
{
}

void ModifierHandle::draw(const mat4& projection_matrix, const mat4& view_matrix)
{
	arrowMesh_LocalX_.draw(projection_matrix, view_matrix * modelMatrixX_, "Smooth Shading");
	arrowMesh_LocalY_.draw(projection_matrix, view_matrix * modelMatrixY_, "Smooth Shading");
	arrowMesh_LocalZ_.draw(projection_matrix, view_matrix * modelMatrixZ_, "Smooth Shading");
}

void ModifierHandle::set_scale(const vec3 scale)
{
	scaleMatrix_ = scaling_matrix(scale);
}

vec3 ModifierHandle::compute_move_vector(const mat4& modelviewProjection, vec2 motion)
{
	vec3 moveAxis;
	if (mode_ == EMode::Translation_X)
		moveAxis = local_x_;
	else if (mode_ == EMode::Translation_Y)
		moveAxis = local_y_;
	else if (mode_ == EMode::Translation_Z)
		moveAxis = local_z_;

	if (mode_ != EMode::None)
	{
		vec4 t = modelviewProjection * vec4(moveAxis, 1.0f);
		vec2 tVec2 = vec2(t[0], -t[1]);
		tVec2.normalize();

		float scalar = pmp::dot(motion, tVec2);
		vec3 movement = moveAxis * scalar * 0.001f;

		return movement;
	}
	else
		return vec3(0, 0, 0);
}

void ModifierHandle::init_local_coordinate_system(mat4 modelview, vec3 normal)
{
	local_z_ = normal;
	vec4 right = vec4(1, 0, 0, 0);
	local_x_ = cross(vec3(right[0], right[1], right[2]), local_z_);
	local_y_ = cross(local_z_, local_x_);
}

void ModifierHandle::set_origin(const vec3& origin, const vec3& normal)
{
	vec3 normalNormalized = normalize(normal) * 0.03f;
	origin_ = origin + normalNormalized;

	precompute_modelViewMatrix();
}

bool ModifierHandle::is_hit(const Ray& ray)
{
	if (is_hit(ray, modelMatrixInverseX_, arrowMesh_LocalX_))
	{
		mode_ = EMode::Translation_X;
		return true;
	}
	else if (is_hit(ray, modelMatrixInverseY_, arrowMesh_LocalY_))
	{
		mode_ = EMode::Translation_Y;
		return true;
	}
	else if (is_hit(ray, modelMatrixInverseZ_, arrowMesh_LocalZ_))
	{
		mode_ = EMode::Translation_Z;
		return true;
	}
	else
	{
		mode_ = EMode::None;
		return false;
	}
}

void ModifierHandle::precompute_modelViewMatrix()
{
	modelMatrixX_ = compute_modelViewMatrix(local_x_);
	modelMatrixInverseX_ = inverse(modelMatrixX_);

	modelMatrixY_ = compute_modelViewMatrix(local_y_);
	modelMatrixInverseY_ = inverse(modelMatrixY_);

	modelMatrixZ_ = compute_modelViewMatrix(local_z_);
	modelMatrixInverseZ_ = inverse(modelMatrixZ_);
}

mat4 ModifierHandle::compute_modelViewMatrix(vec3 forward)
{
	const vec3 forwardN = normalize(forward);
	const vec3 defForward(0.f, 0.f, 1.f);
	const float angle = acos(dot(forwardN, defForward)) * 360.f / (2.f * 3.1415f);
	return translation_matrix(origin_) * transpose(rotation_matrix(cross(forwardN, defForward), angle)) * scaleMatrix_;
}

bool ModifierHandle::is_hit(const Ray& ray, mat4 modelMatrixInverse, const SurfaceColorMesh& mesh) const
{
	Ray localRay;
	const vec4 origin = modelMatrixInverse * pmp::vec4(ray.origin, 1.f);
	const vec4 direction = modelMatrixInverse * pmp::vec4(ray.direction, 0.f);
	localRay.origin = vec3(origin[0], origin[1], origin[2]);
	localRay.direction = vec3(direction[0], direction[1], direction[2]);

	auto triangles = mesh.get_face_property<algorithm::IntersectionTriangle>("f:intersect");

	for (Face f : mesh.faces())
	{
		if (algorithm::intersect(localRay, triangles[f], 10000.f))
		{
			return true;
		}
	}

	return false;
}

void ModifierHandle::precompute_intersection_structure()
{
	precompute_intersection_structure(arrowMesh_LocalX_);
	precompute_intersection_structure(arrowMesh_LocalY_);
	precompute_intersection_structure(arrowMesh_LocalZ_);
}

void ModifierHandle::precompute_intersection_structure(SurfaceColorMesh& mesh)
{
	auto points = mesh.get_vertex_property<Point>("v:point");
	auto triangles = mesh.add_face_property<algorithm::IntersectionTriangle>("f:intersect");

	for (Face f : mesh.faces())
	{
		auto it = mesh.vertices(f).begin();
		const vec3& p0 = points[*it]; ++it;
		const vec3& p1 = points[*it]; ++it;
		const vec3& p2 = points[*it];

		triangles[f] = algorithm::IntersectionTriangle(p0, p1, p2);
	}
}
