#include "HandleMesh.hpp"

HandleMesh::HandleMesh()
{
}

HandleMesh::HandleMesh(Normal normal, vec3 origin)
	: origin_(origin), 
	scaleMatrix_(mat4::identity() * 0.001f)
{
}

HandleMesh::~HandleMesh()
{
}

vec3 HandleMesh::CalcMoveVector(mat4 modelviewProjection, vec2 motion)
{
	vec4 t = modelviewProjection * vec4(currMoveAxis_, 1.0f);
	vec2 tVec2 = vec2(t[0], -t[1]);
	tVec2.normalize();

	float scalar = pmp::dot(motion, tVec2);
	vec3 movement = currMoveAxis_ * scalar * 0.001f;

	return movement;
}

HandleMesh HandleMesh::CreateSimpleMesh(Normal normal, vec3 origin)
{
	HandleMesh resultMesh = HandleMesh(normal, origin);
	bool b = resultMesh.read("../models/arrow.off");
	return resultMesh;
}

const mat4& HandleMesh::GetModelMatrix() const
{
	return modelMatrix_;
}

void HandleMesh::InitLocalCoordinateSystem(mat4 modelview, vec3 normal)
{
	local_z_ = normal;
	vec4 right = vec4(1, 0, 0, 0);
	local_x_ = cross(vec3(right[0], right[1], right[2]), local_z_);
	local_y_ = cross(local_z_, local_x_);

	currMoveAxis_ = local_z_;
}
void HandleMesh::SetLocalMoveAxis(int axis)
{
	if (axis == 0)
		currMoveAxis_ = local_x_;
	if (axis == 1)
		currMoveAxis_ = local_y_;
	if (axis == 2)
		currMoveAxis_ = local_z_;
}

void HandleMesh::SetOrigin(const vec3& origin)
{
	origin_ = origin;
	modelMatrix_ = translation_matrix(origin);
}

void HandleMesh::SetOrientation(const Normal& forward, const Normal& up)
{
	const vec3 forwardN = normalize(forward);
	const vec3 defForward(0.f, 0.f, 1.f);
	const float angle = acos(dot(forwardN, defForward)) * 360.f / (2.f * 3.1415f);
	modelMatrix_ = translation_matrix(origin_) * transpose(rotation_matrix(cross(forwardN, defForward), angle));
}