#include "HandleMesh.hpp"

HandleMesh::HandleMesh()
{
}

HandleMesh::HandleMesh(Normal normal, vec3 origin)
	: origin_(origin), translationMatrix_()
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
	Vertex v0, v1, v2, v3, v4, v5;
	float size = 0.01f;
	v0 = resultMesh.add_vertex(origin);

	vec3 vec = vec3(-normal[1], normal[0], 0);
	vec.normalize();
	v1 = resultMesh.add_vertex(origin + vec * size);

	vec = vec3(normal[1], -normal[0], 0);
	vec.normalize();
	v2 = resultMesh.add_vertex(origin + vec * size);

	//	vec3 cross = vec;

	vec = pmp::cross(normal, vec) * size;
	vec.normalize();
	v3 = resultMesh.add_vertex(origin + vec * size);

	vec = vec3(-vec[0], -vec[1], -vec[2]);
	vec.normalize();
	v4 = resultMesh.add_vertex(origin + vec * size);

	v5 = resultMesh.add_vertex(origin + normal * 0.2f);

	//meshHandle_.add_triangle(v0, v1, v3);
	//meshHandle_.add_triangle(v0, v3, v2);
	//meshHandle_.add_triangle(v0, v2, v4);
	//meshHandle_.add_triangle(v0, v4, v1);
	resultMesh.add_triangle(v1, v3, v5);
	resultMesh.add_triangle(v3, v2, v5);
	resultMesh.add_triangle(v2, v4, v5);
	resultMesh.add_triangle(v4, v1, v5);

	auto vProp = resultMesh.add_vertex_property<Color>("v:col");
	for (auto v : resultMesh.vertices())
		vProp[v] = Color(0.3f, 0.3f, 1);
	return resultMesh;
}

mat4 HandleMesh::GetTranslationMatrix()
{
	return translationMatrix_;
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


void HandleMesh::SetOrigin(vec3 origin)
{
	translationMatrix_ = translation_matrix(origin - origin_);
}
