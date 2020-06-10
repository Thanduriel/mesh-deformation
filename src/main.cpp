#include "pmp/algorithms/SurfaceSmoothing.h"
#include "VertexSelectionViewer.hpp"
#include "utils/PlaneGen.hpp"

// CRT's memory leak detection
#ifndef NDEBUG 
#if defined(_MSC_VER)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif
#endif

int main()
{
	// no leaks should be detected iff the window is closed via the [x] button
	// ESC just calls exit(0) 
#ifndef NDEBUG 
#if defined(_MSC_VER)
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//	_CrtSetBreakAlloc(2760);
#endif
#endif

	using namespace pmp;
	const char* meshFile = "../dependencies/pmp-library/external/pmp-data/off/bunny.off";
	VertexSelectionViewer window("mesh-deformation", 1366, 768);
	auto mesh = util::PlaneGenerator().generate(vec2(10, 10), ivec2(350));
	mesh.write("../models/plane.off");
	window.load_mesh(meshFile);
	window.run();
	
	return 0;
}