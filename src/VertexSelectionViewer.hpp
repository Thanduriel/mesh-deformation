#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>

namespace algorithm {
	class Deformation;
}

class VertexSelectionViewer : public pmp::TrackballViewer { 
public:
	VertexSelectionViewer(const char* title, int width, int height, bool showgui = true);

	~VertexSelectionViewer();

	virtual void draw(const std::string& draw_mode) override;

	//! load a mesh from file \c filename
	bool load_mesh(const char* filename);

	void motion(double xpos, double ypos) override;

	void keyboard(int key, int scancode, int action, int mods) override;

	pmp::Vertex pick_vertex(int x, int y);

	std::vector<Vertex> pick_vertex(int x, int y, float radius);

	void process_imgui() override;

	void update_mesh();

private:
	float brushSize_;
 	std::unique_ptr<algorithm::Deformation> deformationSpace_;
	std::string filename_;	
	bool isVertexTranslationActive_;
	pmp::Normal translationNormal_;

	SurfaceColorMesh mesh_;
	pmp::vec3 pickPosition_;
	pmp::Vertex pickVertex_;


};