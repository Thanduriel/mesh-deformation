#include "VertexSelectionViewer.hpp"
#include "algorithms/deformation.hpp"
#include <pmp/algorithms/SurfaceNormals.h>
#include "utils/validation.hpp"
#include <array>

using namespace pmp;

const std::array<Color, static_cast<size_t>(VertexSelectionViewer::VertexDrawingMode::COUNT)> COLORS =
{
	Color(0.f,0.f,0.f),
	Color(1.f,0.f,0.f),
	Color(0.f,1.f,0.f),
	Color(0.f,0.f,1.f)
};

VertexSelectionViewer::VertexSelectionViewer(const char* title, int width, int height, bool showgui)
	: TrackballViewer(title, width, height, showgui),
	brushSize_(0.05),
	viewerMode_(ViewerMode::View),
	isVertexTranslationMouseActive_(false),
	pickPosition_(0, 0, 0),
	pickVertex_(0)
{
	clear_draw_modes();
	add_draw_mode("Smooth Shading");
	add_draw_mode("Wireframe");
	set_draw_mode("Smooth Shading");
}

VertexSelectionViewer::~VertexSelectionViewer()
{
}

void VertexSelectionViewer::do_processing()
{
	if (isVertexTranslationMouseActive_)
	{
		switch (viewerMode_)
		{
		case ViewerMode::View:
			break;
		case ViewerMode::Translation:
			translationHandle(mousePosX_, mousePosY_);
			break;
		case ViewerMode::Rotation:
			rotationHandle(mousePosX_, mousePosY_);
			break;
		case ViewerMode::Scale:
			scaleHandle(mousePosX_, mousePosY_);
			break;
		default:
			break;
		}
	}
}

void VertexSelectionViewer::draw(const std::string& draw_mode)
{
	if (meshIsDirty_)
	{
		update_mesh();
		meshIsDirty_ = false;
	}
	if (viewerMode_ != ViewerMode::View)
	{
		meshHandle_.draw(projection_matrix_, modelview_matrix_);
	}
	mesh_.draw(projection_matrix_, modelview_matrix_, draw_mode);
}

void VertexSelectionViewer::keyboard(int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS && action != GLFW_REPEAT)
		return;

	if (viewerMode_ == ViewerMode::View)
	{
		switch (key)
		{
		case GLFW_KEY_C: //clear 
		{
			auto vProp = mesh_.get_vertex_property<Color>("v:col");
			for (auto v : mesh_.vertices())
				vProp[v] = COLORS[static_cast<size_t>(VertexDrawingMode::Clear)];

			meshIsDirty_ = true;
			return;
		}
		case GLFW_KEY_W: //support region
			vertexDrawingMode_ = VertexDrawingMode::Support;
			return;
		case GLFW_KEY_Q: //handle region
			vertexDrawingMode_ = VertexDrawingMode::Handle;
			return;
		case GLFW_KEY_E:
			vertexDrawingMode_ = VertexDrawingMode::Clear;
			return;
		case GLFW_KEY_R:
			vertexDrawingMode_ = VertexDrawingMode::None;
			return;
		case GLFW_KEY_SPACE:
			viewerMode_ = ViewerMode::Translation;
			vertexDrawingMode_ = VertexDrawingMode::None;
			meshHandle_.set_translationMode();
			init_modifier();
			return;
		}
	}
	else
	{
		switch (key)
		{
		case GLFW_KEY_R:
			viewerMode_ = ViewerMode::Rotation;
			meshHandle_.set_rotationMode();
			return;
		case GLFW_KEY_3:
			deformationSpace_->set_area_scaling(!deformationSpace_->get_area_scaling());
			meshIsDirty_ = true;
			return;
		case GLFW_KEY_S:
			viewerMode_ = ViewerMode::Scale;
			meshHandle_.set_scaleMode();
			return;
		case GLFW_KEY_SPACE:
			viewerMode_ = ViewerMode::View;
			return;
		case GLFW_KEY_T:
			viewerMode_ = ViewerMode::Translation;
			meshHandle_.set_translationMode();
			return;
		}
	}

	// mode independend inputs
	switch (key)
	{
	case GLFW_KEY_BACKSPACE: // reload model
	{
		load_mesh(filename_.c_str());
		break;
	}
	case GLFW_KEY_1:
		set_draw_mode("Smooth Shading");
		break;
	case GLFW_KEY_2:
		set_draw_mode("Wireframe");
		break;
	default:
		// Q would close the window
		if (key != GLFW_KEY_Q)
			TrackballViewer::keyboard(key, scancode, action, mods);
	}

}

bool VertexSelectionViewer::load_mesh(const char* filename)
{
	// load mesh
	if (mesh_.read(filename))
	{
		// update scene center and bounds
		BoundingBox bb = mesh_.bounds();
		set_scene((vec3)bb.center(), 0.5 * bb.size());
		meshHandle_.set_scale(vec3(bb.size() * 0.1f));

		// compute face & vertex normals, update face indices
		update_mesh();

		// set draw mode
		if (mesh_.n_faces())
		{
			set_draw_mode("Solid Smooth");
		}
		else if (mesh_.n_vertices())
		{
			set_draw_mode("Points");
		}

		// print mesh statistic
		std::cout << "Load " << filename << ": " << mesh_.n_vertices()
			<< " vertices, " << mesh_.n_faces() << " faces\n";
		auto vProp = mesh_.get_vertex_property<Color>("v:col");
		const auto& [face, angle] = util::find_min_angle(mesh_);
		std::cout << angle << "\n";
		for (Vertex v : mesh_.vertices(face))
			vProp[v] = Color(1, 1, 1);
		filename_ = filename;

		// construct modifier
		deformationSpace_ = std::make_unique<algorithm::Deformation>(mesh_);

		return true;
	}

	std::cerr << "Failed to read mesh from " << filename << " !" << std::endl;
	return false;
}

void VertexSelectionViewer::motion(double xpos, double ypos)
{
	mousePosX_ = xpos;
	mousePosY_ = ypos;

	if (!isVertexTranslationMouseActive_)
	{
		if (vertexDrawingMode_ == VertexDrawingMode::None)
			TrackballViewer::motion(xpos, ypos);
		else if (isMouseDown_)
			draw_on_mesh();
	}
}

void VertexSelectionViewer::mouse(int button, int action, int mods)
{
	isMouseDown_ = action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_1;
	if (action == GLFW_PRESS && viewerMode_ != ViewerMode::View
		&& meshHandle_.is_hit(get_ray(last_point_2d_[0], last_point_2d_[1])))
	{
		isVertexTranslationMouseActive_ = true;
	}
	else
	{
		if (vertexDrawingMode_ != VertexDrawingMode::None && viewerMode_ == ViewerMode::View)
			draw_on_mesh();

		isVertexTranslationMouseActive_ = false;
		TrackballViewer::mouse(button, action, mods);
	}
}

Vertex VertexSelectionViewer::pick_vertex(int x, int y)
{
	Vertex vmin;

	vec3 p;
	Scalar d, dmin(FLT_MAX);

	if (TrackballViewer::pick(x, y, p))
	{
		Point picked_position(p);
		for (auto v : mesh_.vertices())
		{
			d = distance(mesh_.position(v), picked_position);
			if (d < dmin)
			{
				dmin = d;
				vmin = v;
			}
		}
	}
	return vmin;
}

std::vector<Vertex> VertexSelectionViewer::pick_vertex(int x, int y, float radius)
{
	std::vector<Vertex> vVector;

	vec3 p;
	Scalar d;

	if (TrackballViewer::pick(x, y, p))
	{
		Point picked_position(p);
		for (auto v : mesh_.vertices())
		{
			d = distance(mesh_.position(v), picked_position);
			if (d < radius)
			{
				vVector.push_back(v);
			}
		}
	}
	return vVector;
}

void VertexSelectionViewer::process_imgui()
{
	if (ImGui::CollapsingHeader("Selection", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::CollapsingHeader("Help", ImGuiTreeNodeFlags_None))
		{
			ImGui::BulletText("Space: Activate Modifier");
			ImGui::BulletText("Q: Draw handle region");
			ImGui::BulletText("W: Draw support region");
			ImGui::BulletText("E: Draw default region");
		}
		if (viewerMode_ != ViewerMode::View)
		{
			ImGui::BulletText("X: Select local x-axis");
			ImGui::BulletText("Y: Select local y-axis");
			ImGui::BulletText("Z: Select local z-axis");
			ImGui::Separator();
			ImGui::Text(("Current ViewMode " + viewerMode_).c_str());
		}
		const BoundingBox bb = mesh_.bounds();
		ImGui::SliderFloat("BrushSize", &brushSize_, 0.01, bb.size() * 0.5);
	}
	if (deformationSpace_->is_set() && ImGui::CollapsingHeader("Operator", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::SliderInt("Order", &operatorOrder_, 1, 3))
		{
			deformationSpace_->set_order(operatorOrder_);
			meshIsDirty_ = true;
		}
		if (ImGui::SliderFloat("smoothness", &smoothness_, 0.f, 2.f))
		{
			deformationSpace_->set_smoothness_handle(smoothness_);
			meshIsDirty_ = true;
		}
		if (ImGui::Checkbox("area scaling", &useAreaScaling_))
		{
			deformationSpace_->set_area_scaling(useAreaScaling_);
			meshIsDirty_ = true;
		}
	}
}

void VertexSelectionViewer::update_mesh()
{
	// update scene center and radius, but don't update camera view
	BoundingBox bb = mesh_.bounds();
	center_ = (vec3)bb.center();
	radius_ = 0.5f * bb.size();

	if (viewerMode_ != ViewerMode::View)
	{
		meshHandle_.set_origin(translationPoint_, translationNormal_);
	}

	// re-compute face and vertex normals
	mesh_.update_opengl_buffers();
}

void VertexSelectionViewer::translationHandle(float xpos, float ypos)
{
	vec2 currMousePos = vec2(xpos, ypos);
	vec2 mouseMotion = currMousePos - vec2(last_point_2d_[0], last_point_2d_[1]);
	float mouseMotionNorm = pmp::norm(mouseMotion);
	if ((ypos > 0 || xpos > 0) && mouseMotionNorm > 0)
	{
		vec3 movement = meshHandle_.compute_move_vector(projection_matrix_ * modelview_matrix_, mouseMotion);
		deformationSpace_->translate(movement);
		translationPoint_ += movement;
		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ = true;
	}
}

void VertexSelectionViewer::rotationHandle(float xpos, float ypos)
{
	vec2 midScreen = vec2(width() / 2.0f, height() / 2.0f);
	vec2 currMousePos = vec2(xpos, ypos) - midScreen;
	vec2 lastPos = vec2(last_point_2d_[0], last_point_2d_[1]) - midScreen;
	vec2 mouseMotion = currMousePos - vec2(last_point_2d_[0], last_point_2d_[1]);
	float mouseMotionNorm = pmp::norm(mouseMotion);

	currMousePos.normalize();
	lastPos.normalize();

	if (mouseMotionNorm > 0)
	{
		float angle = (atan2(lastPos[1], lastPos[0]) - atan2(currMousePos[1], currMousePos[0])) * 180.0f / M_PI;
		deformationSpace_->rotate(meshHandle_.compute_rotation_vector(), angle);
		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ = true;
	}
}

void VertexSelectionViewer::scaleHandle(float xpos, float ypos)
{
	vec2 midScreen = vec2(width() / 2.0f, height() / 2.0f);
	vec2 currMousePos = vec2(xpos, ypos) - midScreen;
	vec2 lastPos = vec2(last_point_2d_[0], last_point_2d_[1]) - midScreen;

	float mouseMotionNorm = norm(currMousePos) - norm(lastPos);
	if (mouseMotionNorm != 0)
	{
		std::cout << mouseMotionNorm << std::endl;
		deformationSpace_->scale(1 + mouseMotionNorm * 0.001f);
		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ = true;
	}
}

Ray VertexSelectionViewer::get_ray(int x, int y)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// take into accout highDPI scaling
	x *= high_dpi_scaling();
	y *= high_dpi_scaling();

	// in OpenGL y=0 is at the 'bottom'
	y = viewport[3] - y;

	const float xf = ((float)x - (float)viewport[0]) / ((float)viewport[2]) * 2.0f - 1.0f;
	const float yf = ((float)y - (float)viewport[1]) / ((float)viewport[3]) * 2.0f - 1.0f;

	const mat4 mvp = projection_matrix_ * modelview_matrix_;
	const mat4 inv = inverse(mvp);
	// far plane
	vec4 p = inv * vec4(xf, yf, 1.f, 1.0f);
	// near plane
	vec4 origin = inv * vec4(xf, yf, 0.f, 1.0f);
	p /= p[3];
	origin /= origin[3];
	const vec4 dir = p - origin;

	return { vec3(origin[0], origin[1], origin[2]), normalize(vec3(dir[0], dir[1], dir[2])) };

}

void VertexSelectionViewer::init_modifier()
{
	auto points = mesh_.get_vertex_property<Point>("v:point");
	auto colors = mesh_.get_vertex_property<Color>("v:col");
	std::vector<Vertex> supportVertices;
	std::vector<Vertex> handleVertices;
	pmp::Normal normal(0, 0, 0);
	pmp::Point handlePoint(0, 0, 0);
	int pointIndex = 0;

	for (Vertex v : mesh_.vertices())
	{
		if (colors[v] == Color(0, 1, 0))
		{
			handleVertices.push_back(v);
			normal += SurfaceNormals::compute_vertex_normal(mesh_, v);
			handlePoint += points[v];
			pointIndex++;
		}
		else if (colors[v] == Color(0, 0, 1))
		{
			supportVertices.push_back(v);
		}
	}

	translationPoint_ = handlePoint / pointIndex;
	normal.normalize();
	translationNormal_ = normal;

	deformationSpace_->set_regions(supportVertices, handleVertices);

	meshHandle_.init_local_coordinate_system(modelview_matrix_, translationNormal_);
	viewerMode_ = ViewerMode::Translation;
	meshHandle_.set_translationMode();
	meshIsDirty_ = true;
}

void VertexSelectionViewer::draw_on_mesh()
{
	TrackballViewer::pick(pickPosition_);
	double x = 0;
	double y = 0;
	cursor_pos(x, y);
	auto vVector = pick_vertex(x, y, brushSize_);

	if (!vVector.empty())
	{
		auto vProp = mesh_.get_vertex_property<Color>("v:col");
		for (auto v : vVector)
			vProp[v] = COLORS[static_cast<size_t>(vertexDrawingMode_)];
		meshIsDirty_ = true;
	}
}