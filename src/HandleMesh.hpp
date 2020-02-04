#ifndef M_PI
#define M_PI 3.1415 // expected by pmp
#endif // !M_PI
#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>
#include <memory>

namespace algorithm {
	class Deformation;
}

class HandleMesh : public SurfaceColorMesh {
public:
	HandleMesh();
	HandleMesh(Normal normal, vec3 origin);
	~HandleMesh();

	vec3 CalcMoveVector(mat4 modelviewProjection, vec2 motion);
	static HandleMesh CreateSimpleMesh(Normal normal, vec3 origin);
	const mat4& GetModelMatrix() const;
	void SetOrigin(const vec3& origin);
	void SetOrientation(const Normal& forward, const Normal& up);
	void InitLocalCoordinateSystem(mat4 modelview, vec3 normal);
	void SetLocalMoveAxis(int axis);
private:
	vec3 origin_;
	mat4 modelMatrix_;
	mat4 scaleMatrix_;
	vec3 local_x_;
	vec3 local_y_;
	vec3 local_z_;

	vec3 currMoveAxis_;
};