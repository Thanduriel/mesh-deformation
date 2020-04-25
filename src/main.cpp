#include "pmp/visualization/MeshViewer.h"
#include "pmp/algorithms/SurfaceSmoothing.h"
#include "VertexSelectionViewer.hpp"
#include "utils/planegen.hpp"

int main()
{
	using namespace pmp;
	const char* meshFile = "../dependencies/pmp-library/external/pmp-data/off/bunny.off";
	VertexSelectionViewer window("mesh-deformation", 1366, 768);
	window.load_mesh(meshFile);
	window.run();
	
	return 0;
}