#include "ModifierHandle.hpp"
#include "algorithms/intersection.hpp"
#include <iostream>

ModifierHandle::ModifierHandle()
{
	arrowMesh_LocalX_.read("../models/arrow.off");
	arrowMesh_LocalX_.update_opengl_buffers();
	arrowMesh_LocalX_.update_color_buffer();

	arrowMesh_LocalY_.read("../models/arrow.off");
	arrowMesh_LocalY_.update_opengl_buffers();
	arrowMesh_LocalY_.update_color_buffer();

	arrowMesh_LocalZ_.read("../models/arrow.off");
	arrowMesh_LocalZ_.update_opengl_buffers();
	arrowMesh_LocalZ_.update_color_buffer();

	torusMesh_RotationX_.read("../models/torus.obj");
	torusMesh_RotationX_.update_opengl_buffers();
	torusMesh_RotationX_.update_color_buffer();

	torusMesh_RotationY_.read("../models/torus.obj");
	torusMesh_RotationY_.update_opengl_buffers();
	torusMesh_RotationY_.update_color_buffer();

	torusMesh_RotationZ_.read("../models/torus.obj");
	torusMesh_RotationZ_.update_opengl_buffers();
	torusMesh_RotationZ_.update_color_buffer();

	scaleMesh_ScaleX_.read("../models/scaleMesh.off");
	scaleMesh_ScaleX_.update_opengl_buffers();
	scaleMesh_ScaleX_.update_color_buffer();

	scaleMesh_ScaleY_.read("../models/scaleMesh.off");
	scaleMesh_ScaleY_.update_opengl_buffers();
	scaleMesh_ScaleY_.update_color_buffer();

	scaleMesh_ScaleZ_.read("../models/scaleMesh.off");
	scaleMesh_ScaleZ_.update_opengl_buffers();
	scaleMesh_ScaleZ_.update_color_buffer();

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
		//scaleMesh_ScaleZ_.draw(projection_matrix, view_matrix * modelMatrixScaleZ_, "Smooth Shading");
	}
}

void ModifierHandle::set_scale(const vec3 scale)
{
	scaleMatrixArrow_ = scaling_matrix(scale);
}

void ModifierHandle::set_scale(float scale)
{
	const float scaleArrow = 1.0f;
	const float scaleTorus = 0.5f;
	const float scaleScale = 0.05f;


	scaleMatrixArrow_ = scaling_matrix(scaleArrow * scale);
	scaleMatrixTorus_ = scaling_matrix(scaleTorus * scale);
	scaleMatrixScale_ = scaling_matrix(scaleScale * scale);
}

bool ModifierHandle::is_translationMode() const
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

bool ModifierHandle::is_rotationMode() const
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

bool ModifierHandle::is_scaleMode() const
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

vec3 ModifierHandle::get_active_translation_axis() const
{
	if (mode_ == EMode::Translation_X)
		return local_x_;
	else if (mode_ == EMode::Translation_Y)
		return local_y_;
	else if (mode_ == EMode::Translation_Z)
		return local_z_;
	else
		return vec3(0, 0, 0);
}

std::vector<vec3> ModifierHandle::get_active_normal_axes()
{
	std::vector<vec3> result;
	if (mode_ == EMode::Translation_X)
	{
		result.push_back(local_y_);
		result.push_back(local_z_);
	}
	else if (mode_ == EMode::Translation_Y)
	{
		result.push_back(local_z_);
		result.push_back(local_x_);
	}
	else if (mode_ == EMode::Translation_Z)
	{
		result.push_back(local_x_);
		result.push_back(local_y_);
	}

	return result;
}

vec3 ModifierHandle::get_last_hit_point()
{
	return last_hit_point_;
}

vec3 ModifierHandle::compute_move_vector(float scalar)
{
	vec3 moveAxis = get_active_translation_axis();
	return moveAxis * scalar;
}

vec3 ModifierHandle::compute_move_vector()
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

	return origin_ + moveAxis;
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

vec3 ModifierHandle::origin()
{
	return origin_;
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
	vec3 newOrigin = origin + normal;
	vec3 dist = newOrigin - origin_;
	last_hit_point_ += dist;
	origin_ = newOrigin;

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

void ModifierHandle::set_mouseStartPos(vec2 pos)
{
	mouseStartPos_ = pos;
}

vec2 ModifierHandle::get_mouseStartPos()
{
	return mouseStartPos_;
}

bool ModifierHandle::is_hit(const Ray& ray)
{
	remove_selection();

	std::optional<float> rayResult = std::nullopt;
	std::optional<float> rayResultHelp = std::nullopt;
	bool hit = false;

	if (is_translationMode())
	{
		if (rayResultHelp = is_hit(ray, modelMatrixInverseX_, arrowMesh_LocalX_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Translation_X;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
		if (rayResultHelp = is_hit(ray, modelMatrixInverseY_, arrowMesh_LocalY_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Translation_Y;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
		if (rayResultHelp = is_hit(ray, modelMatrixInverseZ_, arrowMesh_LocalZ_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Translation_Z;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
	}
	else if (is_rotationMode())
	{
		if (rayResultHelp = is_hit(ray, modelMatrixInverseRotationX_, torusMesh_RotationX_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Rotation_X;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
		if (rayResultHelp = is_hit(ray, modelMatrixInverseRotationY_, torusMesh_RotationY_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Rotation_Y;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
		if (rayResultHelp = is_hit(ray, modelMatrixInverseRotationZ_, torusMesh_RotationZ_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Rotation_Z;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
	}
	else if (is_scaleMode())
	{
		if (rayResultHelp = is_hit(ray, modelMatrixScaleInverseX_, scaleMesh_ScaleX_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Scale_X;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
		if (rayResultHelp = is_hit(ray, modelMatrixScaleInverseY_, scaleMesh_ScaleY_))
		{
			if (!rayResult.has_value() || rayResultHelp.value() < rayResult.value())
			{
				mode_ = EMode::Scale_Y;
				rayResult = rayResultHelp;
				hit = true;
			}
		}
	}

	if (hit)
	{
		switch (mode_)
		{
		case ModifierHandle::EMode::None:
			break;
		case ModifierHandle::EMode::Translation_X:
			set_selection(arrowMesh_LocalX_);
			break;
		case ModifierHandle::EMode::Translation_Y:
			set_selection(arrowMesh_LocalY_);
			break;
		case ModifierHandle::EMode::Translation_Z:
			set_selection(arrowMesh_LocalZ_);
			break;
		case ModifierHandle::EMode::Rotation_X:
			set_selection(torusMesh_RotationX_);
			break;
		case ModifierHandle::EMode::Rotation_Y:
			set_selection(torusMesh_RotationY_);
			break;
		case ModifierHandle::EMode::Rotation_Z:
			set_selection(torusMesh_RotationZ_);
			break;
		case ModifierHandle::EMode::Scale_X:
			set_selection(scaleMesh_ScaleX_);
			break;
		case ModifierHandle::EMode::Scale_Y:
			set_selection(scaleMesh_ScaleY_);
			break;
		case ModifierHandle::EMode::Scale_Z:
			break;
		default:
			break;
		}
	}
	if (rayResult)
	{
		std::cout << "True" << std::endl;
		last_hit_point_ = ray.origin + ray.direction * rayResult.value();
		return true;
	}
	else
	{
		std::cout << "False" << std::endl;
		return false;
	}
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
	const float angle = acos(dot(forwardN, defForward)) * 180.f / M_PI;
	return translation_matrix(origin_) * transpose(rotation_matrix(cross(forwardN, defForward), angle)) * scaleMatrix;
}

std::optional<float> ModifierHandle::is_hit(const Ray& ray, mat4 modelMatrixInverse, const SurfaceColorMesh& mesh) const
{
	Ray localRay;
	const vec4 origin = modelMatrixInverse * pmp::vec4(ray.origin, 1.f);
	const vec4 direction = modelMatrixInverse * pmp::vec4(ray.direction, 0.f);
	localRay.origin = vec3(origin[0], origin[1], origin[2]);
	localRay.direction = vec3(direction[0], direction[1], direction[2]);

	auto triangles = mesh.get_face_property<algorithm::IntersectionTriangle>("f:intersect");

	for (Face f : mesh.faces())
	{
		auto scalar = algorithm::intersect(localRay, triangles[f]);
		if (scalar)
		{
			return scalar;
		}
	}

	return std::nullopt;
}

void ModifierHandle::set_selection(SurfaceColorMesh& mesh)
{
	if (selectionMeshes_.find(&mesh) == selectionMeshes_.end())
	{
		auto vProp = mesh.get_vertex_property<Color>("v:col");

		for (Vertex v : mesh.vertices())
			vProp[v] = Color(1.0f, 1.0f, 0.0f);
		selectionMeshes_.insert(&mesh);
		mesh.update_color_buffer();
	}
}

void ModifierHandle::remove_selection()
{
	remove_selection(arrowMesh_LocalX_);
	remove_selection(arrowMesh_LocalY_);
	remove_selection(arrowMesh_LocalZ_);
	remove_selection(torusMesh_RotationX_);
	remove_selection(torusMesh_RotationY_);
	remove_selection(torusMesh_RotationZ_);
	remove_selection(scaleMesh_ScaleX_);
	remove_selection(scaleMesh_ScaleY_);
}

void ModifierHandle::remove_selection(SurfaceColorMesh& mesh)
{
	if (selectionMeshes_.find(&mesh) != selectionMeshes_.end())
	{
		auto vProp = mesh.get_vertex_property<Color>("v:col");

		for (Vertex v : mesh.vertices())
			vProp[v] = Color(1.0f, 0.0f, 0.0f);

		mesh.update_color_buffer();
		selectionMeshes_.erase(&mesh);
	}
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
