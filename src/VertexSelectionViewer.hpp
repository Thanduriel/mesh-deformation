#include "pmp/visualization/MeshViewer.h"
#include "imgui.h"
#include <SurfaceColorMesh.hpp>
#include "ModifierHandle.hpp"
#include "utils/octree.hpp"
#include <memory>

namespace algorithm {
	class Deformation;
}

enum class ViewerMode
{
	View,
	Translation,
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
	case ViewerMode::Translation:
		os += "Translation";
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
	enum struct VertexDrawingMode
	{
		None,
		Clear,
		Handle,
		Support,
		COUNT
	};

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

	// Picks all vertices in a radius around the position on the mesh.
	// @param onlyConnected Starts with the closest vertex and adds only 
	//                      those vertices connected to it within the sphere.
	std::vector<Vertex> pick_vertex(int x, int y, float radius, bool onlyConnected = false);

	void process_imgui() override;

	void update_mesh();

	void scroll(double, double yoffset) override;

	// Compute ray going through a screen position.
	Ray get_ray(int x, int y);
private:
	void translationHandle(float xpos, float ypos);
	void rotationHandle(float xpos, float ypos);
	void scaleHandle(float xpos, float ypos);
	void setHandleOrigin();
	void setHandleScale();
	// @return Whether the current colored areas are valid.
	bool init_modifier();
	void init_picking();
	void draw_on_mesh();
	void set_viewer_mode(ViewerMode mode);

	vec2 compute_screenCoordinates(vec3 vec);
	vec3 compute_WorldCoordinates(vec2 vec, float zf);

	struct SphereQuery
	{
		bool descend(const pmp::vec3& center, double size) const;
		void process(const pmp::vec3& key, Vertex v);

		pmp::vec3 center_;
		float radius_;
		float radiusSq_;
		std::vector<Vertex> verticesHit;
	};

	// gui options
	VertexDrawingMode vertexDrawingMode_ = VertexDrawingMode::None;
	float brushSize_;
	float detailStrength_ = 0.001;
	bool showDetails_ = true;
	int operatorOrder_ = 3;
	int smoothingOrder_ = 1;
	float smoothnessHandle_ = 2.f;
	float smoothnessBoundary_ = 2.f;
	bool useAreaScaling_ = false;
	const char* currentVertexDrawItem_ = nullptr;
	const char* currentModifierItem_ = nullptr;
	char fileNameBuffer_[512] = {};

	util::Octree<pmp::Vertex, 4> queryTree_;
	std::unique_ptr<algorithm::Deformation> deformationSpace_;
	std::string filename_;
	ViewerMode viewerMode_;
	bool isVertexTranslationMouseActive_;
	bool isMouseDown_ = false;
	pmp::Normal translationNormal_;
	pmp::Point translationPoint_;
	
	SurfaceColorMesh mesh_;
	ModifierHandle meshHandle_;
	pmp::vec3 pickPosition_;
	pmp::Vertex pickVertex_;

	enum MeshUpdate
	{
		Geometry = 1,
		VertexColor = 2
	};
	unsigned meshIsDirty_ = 0u;

	double mousePosX_ = 0.0;
	double mousePosY_ = 0.0;


};