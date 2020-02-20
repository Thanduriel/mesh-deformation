#include "pmp/visualization/MeshViewer.h"
#include "pmp/algorithms/SurfaceSmoothing.h"
#include "VertexSelectionViewer.hpp"
#include "utils/planegen.hpp"

int main()
{
	using namespace pmp;
//	util::PlaneGenerator gen;
//	SurfaceMesh m = gen.generate(vec2(1.f,1.f), ivec2(128,128), 0.3f);
//	m.write("plane_rand.off");
//	return 0;
//	const char* meshFile = "../dependencies/pmp-library/external/pmp-data/off/elephant.off";
	const char* meshFile = "../models/plane.off";
	VertexSelectionViewer window("hello world! Test", 1366, 768);
	window.load_mesh(meshFile);
	window.run();
	return 0;
}