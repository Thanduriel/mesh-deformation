#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>
#include "ModifierHandle.hpp"
#include <memory>

namespace algorithm {
	class Deformation;
}

enum class ViewerMode
{
	View,
	Translation_X,
	Translation_Y,
	Translation_Z,
	Rotation,
	Scale
};

inline std::string operator+(std::string os, ViewerMode c)
{
	switch (c)
	{
	case ViewerMode::View: 
		os += "View";
		break;
	case ViewerMode::Translation_X: 
		os += "Translation_X";
		break;
	case ViewerMode::Translation_Y: 
		os += "Translation_Y";
		break;
	case ViewerMode::Translation_Z: 
		os += "Translation_Z";
		break;
	case ViewerMode::Scale: 
		os += "Scale";
		break;
	case ViewerMode::Rotation: 
		os += "Rotation";
		break;
	default:
		os = "None";
	}

	return os;
}

class VertexSelectionViewer : public pmp::TrackballViewer {
public:
	VertexSelectionViewer(const char* title, int width, int height, bool showgui = true);

	~VertexSelectionViewer();

	void do_processing() override;

	void draw(const std::string& draw_mode) override;

	//! load a mesh from file \c filename
	bool load_mesh(const char* filename);

	void motion(double xpos, double ypos) override;

	void mouse(int button, int action, int mods) override;

	void keyboard(int key, int scancode, int action, int mods) override;

	pmp::Vertex pick_vertex(int x, int y);

	std::vector<Vertex> pick_vertex(int x, int y, float radius);

	void process_imgui() override;

	void update_mesh();

	// Compute ray going through a screen position.
	Ray get_ray(int x, int y);
private:
	void translationHandle(float xpos, float ypos);
	void rotationHandle(float xpos, float ypos);
	void scaleHandle(float xpos, float ypos);

	float brushSize_;
	int operatorOrder_ = 3;
	float smoothness_ = 2.f;
	bool useAreaScaling_ = false;
	std::unique_ptr<algorithm::Deformation> deformationSpace_;
	std::string filename_;
	ViewerMode viewerMode_;
	bool isVertexTranslationMouseActive_;
	pmp::Normal translationNormal_;
	pmp::Point translationPoint_;

	SurfaceColorMesh mesh_;
	ModifierHandle meshHandle_;
	pmp::vec3 pickPosition_;
	pmp::Vertex pickVertex_;
	bool meshIsDirty_ = true;


};