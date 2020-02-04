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
	mat4 GetTranslationMatrix();
	void InitLocalCoordinateSystem(mat4 modelview, vec3 normal);
	void SetLocalMoveAxis(int axis);
	void SetOrigin(vec3 origin);
private:
	vec3 origin_;
	mat4 translationMatrix_;

	vec3 local_x_;
	vec3 local_y_;
	vec3 local_z_;

	vec3 currMoveAxis_;
};