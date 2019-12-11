#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>

class VertexSelectionViewer : public pmp::TrackballViewer { 
public:
	VertexSelectionViewer(const char* title, int width, int height, bool showgui = true);

	~VertexSelectionViewer();

	virtual void draw(const std::string& draw_mode) override;

	//! load a mesh from file \c filename
	bool load_mesh(const char* filename);

	void keyboard(int key, int scancode, int action, int mods) override;

	pmp::Vertex pick_vertex(int x, int y);

	void process_imgui() override;

	void update_mesh();

private:
	SurfaceColorMesh mesh_;
	pmp::vec3 pickPosition_;
	pmp::Vertex pickVertex_;

	std::string filename_;	
};