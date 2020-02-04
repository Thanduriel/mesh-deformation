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
	HandleMesh(vec3 origin);
	~HandleMesh();

	static HandleMesh CreateSimpleMesh(Normal normal, vec3 origin);
	mat4 GetTranslationMatrix();
	void SetOrigin(vec3 origin);
private:
	vec3 origin_;
	mat4 translationMatrix_;

};