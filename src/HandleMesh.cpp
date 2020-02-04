#include "HandleMesh.hpp"

HandleMesh::HandleMesh()
{
}

HandleMesh::HandleMesh(vec3 origin) : origin_(origin), translationMatrix_()
{
}

HandleMesh::~HandleMesh()
{
}

HandleMesh HandleMesh::CreateSimpleMesh(Normal normal, vec3 origin)
{
	HandleMesh resultMesh = HandleMesh(origin);
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


void HandleMesh::SetOrigin(vec3 origin)
{
	translationMatrix_ = translation_matrix(origin - origin_);
}
