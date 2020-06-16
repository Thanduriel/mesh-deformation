#include "pmp/algorithms/SurfaceSmoothing.h"
#include "VertexSelectionViewer.hpp"

// CRT's memory leak detection
#ifndef NDEBUG 
#if defined(_MSC_VER)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif
#endif

int main(int paraNum, char* arg[])
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
	if (paraNum > 1)
	{
		meshFile = arg[1];
	}

	VertexSelectionViewer window("mesh-deformation", 1366, 768);
	window.load_mesh(meshFile);
	window.run();
	
	return 0;
}