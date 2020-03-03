#include "ModifierHandle.hpp"
#include "algorithms/intersection.hpp"
#include <iostream>

ModifierHandle::ModifierHandle()
	: scaleMatrixArrow_(scaling_matrix(1.f)), scaleMatrixTorus_(scaling_matrix(0.13f)), scaleMatrixScale_(scaling_matrix(0.01f))
{
	arrowMesh_LocalX_.read("../models/arrow.off");
	arrowMesh_LocalX_.update_opengl_buffers();

	arrowMesh_LocalY_.read("../models/arrow.off");
	arrowMesh_LocalY_.update_opengl_buffers();

	arrowMesh_LocalZ_.read("../models/arrow.off");
	arrowMesh_LocalZ_.update_opengl_buffers();

	torusMesh_RotationX_.read("../models/torus.obj");
	torusMesh_RotationX_.update_opengl_buffers();

	torusMesh_RotationY_.read("../models/torus.obj");
	torusMesh_RotationY_.update_opengl_buffers();

	torusMesh_RotationZ_.read("../models/torus.obj");
	torusMesh_RotationZ_.update_opengl_buffers();

	scaleMesh_ScaleX_.read("../models/arrow.off");
	scaleMesh_ScaleX_.update_opengl_buffers();

	scaleMesh_ScaleY_.read("../models/arrow.off");
	scaleMesh_ScaleY_.update_opengl_buffers();

	scaleMesh_ScaleZ_.read("../models/arrow.off");
	scaleMesh_ScaleZ_.update_opengl_buffers();

	precompute_modelViewMatrix();
	precompute_intersection_structure();
}

ModifierHandle::~ModifierHandle()
{
}

void ModifierHandle::draw(const mat4& projection_matrix, const mat4& view_matrix)
{
	if (is_translationMode())
	{
		arrowMesh_LocalX_.draw(projection_matrix, view_matrix * modelMatrixX_, "Smooth Shading");
		arrowMesh_LocalY_.draw(projection_matrix, view_matrix * modelMatrixY_, "Smooth Shading");
		arrowMesh_LocalZ_.draw(projection_matrix, view_matrix * modelMatrixZ_, "Smooth Shading");
	}
	if (is_rotationMode())
	{
		torusMesh_RotationX_.draw(projection_matrix, view_matrix * modelMatrixRotationX_, "Smooth Shading");
		torusMesh_RotationY_.draw(projection_matrix, view_matrix * modelMatrixRotationY_, "Smooth Shading");
		torusMesh_RotationZ_.draw(projection_matrix, view_matrix * modelMatrixRotationZ_, "Smooth Shading");
	}
	if (is_scaleMode())
	{
		scaleMesh_ScaleX_.draw(projection_matrix, view_matrix * modelMatrixScaleX_, "Smooth Shading");
		scaleMesh_ScaleY_.draw(projection_matrix, view_matrix * modelMatrixScaleY_, "Smooth Shading");
		scaleMesh_ScaleZ_.draw(projection_matrix, view_matrix * modelMatrixScaleZ_, "Smooth Shading");
	}
}

void ModifierHandle::set_scale(const vec3 scale)
{
	scaleMatrixArrow_ = scaling_matrix(scale);
}

bool ModifierHandle::is_translationMode()
{
	switch (mode_)
	{
	case EMode::Translation_X:
	case EMode::Translation_Y:
	case EMode::Translation_Z:
		return true;
	default:
		break;
	}

	return false;
}

bool ModifierHandle::is_rotationMode()
{
	switch (mode_)
	{
	case EMode::Rotation_X:
	case EMode::Rotation_Y:
	case EMode::Rotation_Z:
		return true;
	default:
		break;
	}

	return false;
}

bool ModifierHandle::is_scaleMode()
{
	switch (mode_)
	{
	case EMode::Scale_X:
	case EMode::Scale_Y:
	case EMode::Scale_Z:
		return true;
	default:
		break;
	}

	return false;
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
	else
		return vec3(0, 0, 0);

	vec4 t = modelviewProjection * vec4(moveAxis, 1.0f);
	vec2 tVec2 = vec2(t[0], -t[1]);
	tVec2.normalize();

	float scalar = pmp::dot(motion, tVec2);
	vec3 movement = moveAxis * scalar * 0.001f;

	return movement;
}

vec3 ModifierHandle::compute_rotation_vector()
{
	if (mode_ == EMode::Rotation_X)
		return normalize(local_x_);
	else if (mode_ == EMode::Rotation_Y)
		return normalize(local_y_);
	else if (mode_ == EMode::Rotation_Z)
		return normalize(local_z_);
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

void ModifierHandle::set_translationMode()
{
	switch (mode_)
	{
	case EMode::Translation_X:
	case EMode::Translation_Y:
	case EMode::Translation_Z:
		break;
	default:
		mode_ = EMode::Translation_X;
	}
}

void ModifierHandle::set_rotationMode()
{
	switch (mode_)
	{
	case EMode::Rotation_X:
	case EMode::Rotation_Y:
	case EMode::Rotation_Z:
		break;
	default:
		mode_ = EMode::Rotation_X;
	}
}

void ModifierHandle::set_scaleMode()
{
	switch (mode_)
	{
	case EMode::Scale_X:
	case EMode::Scale_Y:
	case EMode::Scale_Z:
		break;
	default:
		mode_ = EMode::Scale_X;
	}
}

bool ModifierHandle::is_hit(const Ray& ray)
{
	if (is_translationMode())
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
	}
	else if (is_rotationMode())
	{
		if (is_hit(ray, modelMatrixInverseRotationX_, torusMesh_RotationX_))
		{
			mode_ = EMode::Rotation_X;
			return true;
		}
		else if (is_hit(ray, modelMatrixInverseRotationY_, torusMesh_RotationY_))
		{
			mode_ = EMode::Rotation_Y;
			return true;
		}
		else if (is_hit(ray, modelMatrixInverseRotationZ_, torusMesh_RotationZ_))
		{
			mode_ = EMode::Rotation_Z;
			return true;
		}
	}
	else if (is_scaleMode())
	{
		if (is_hit(ray, modelMatrixScaleInverseX_, scaleMesh_ScaleX_))
		{
			mode_ = EMode::Scale_X;
			return true;
		}
		else if (is_hit(ray, modelMatrixScaleInverseY_, scaleMesh_ScaleY_))
		{
			mode_ = EMode::Scale_Y;
			return true;
		}
		else if (is_hit(ray, modelMatrixScaleInverseZ_, scaleMesh_ScaleZ_))
		{
			mode_ = EMode::Scale_Z;
			return true;
		}
	}


	return false;
}

void ModifierHandle::precompute_modelViewMatrix()
{
	modelMatrixX_ = compute_modelViewMatrix(local_x_, scaleMatrixArrow_);
	modelMatrixInverseX_ = inverse(modelMatrixX_);

	modelMatrixY_ = compute_modelViewMatrix(local_y_, scaleMatrixArrow_);
	modelMatrixInverseY_ = inverse(modelMatrixY_);

	modelMatrixZ_ = compute_modelViewMatrix(local_z_, scaleMatrixArrow_);
	modelMatrixInverseZ_ = inverse(modelMatrixZ_);

	modelMatrixRotationX_ = compute_modelViewMatrix(local_x_, scaleMatrixTorus_);
	modelMatrixInverseRotationX_ = inverse(modelMatrixRotationX_);

	modelMatrixRotationY_ = compute_modelViewMatrix(local_y_, scaleMatrixTorus_);
	modelMatrixInverseRotationY_ = inverse(modelMatrixRotationY_);

	modelMatrixRotationZ_ = compute_modelViewMatrix(local_z_, scaleMatrixTorus_);
	modelMatrixInverseRotationZ_ = inverse(modelMatrixRotationZ_);

	modelMatrixScaleX_ = compute_modelViewMatrix(local_x_, scaleMatrixScale_);
	modelMatrixScaleInverseX_ = inverse(modelMatrixScaleX_);

	modelMatrixScaleY_ = compute_modelViewMatrix(local_y_, scaleMatrixScale_);
	modelMatrixScaleInverseY_ = inverse(modelMatrixScaleY_);

	modelMatrixScaleZ_ = compute_modelViewMatrix(local_z_, scaleMatrixScale_);
	modelMatrixScaleInverseZ_ = inverse(modelMatrixScaleZ_);
}

mat4 ModifierHandle::compute_modelViewMatrix(vec3 forward, mat4 scaleMatrix)
{
	const vec3 forwardN = normalize(forward);
	const vec3 defForward(0.f, 0.f, 1.f);
	const float angle = acos(dot(forwardN, defForward)) * 360.f / (2.f * 3.1415f);
	return translation_matrix(origin_) * transpose(rotation_matrix(cross(forwardN, defForward), angle)) * scaleMatrix;
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
	precompute_intersection_structure(torusMesh_RotationX_);
	precompute_intersection_structure(torusMesh_RotationY_);
	precompute_intersection_structure(torusMesh_RotationZ_);
	precompute_intersection_structure(scaleMesh_ScaleX_);
	precompute_intersection_structure(scaleMesh_ScaleY_);
	precompute_intersection_structure(scaleMesh_ScaleZ_);
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
