#include "VertexSelectionViewer.hpp"
#include "algorithms/deformation.hpp"
#include <pmp/algorithms/SurfaceNormals.h>
#include <array>
#include <chrono>

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
	if (meshIsDirty_ & MeshUpdate::Geometry)
		update_mesh();
	if (meshIsDirty_ & MeshUpdate::VertexColor)
		mesh_.update_color_buffer();

	meshIsDirty_ = 0u;

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

			meshIsDirty_ |= MeshUpdate::VertexColor;
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
			if (init_modifier())
			{
				set_viewer_mode(ViewerMode::Translation);
			}
			return;
		}
	}
	else
	{
		switch (key)
		{
		case GLFW_KEY_R:
			set_viewer_mode(ViewerMode::Rotation);
			return;
		case GLFW_KEY_3:
			deformationSpace_->set_area_scaling(!deformationSpace_->get_area_scaling());
			meshIsDirty_ |= MeshUpdate::Geometry;
			return;
		case GLFW_KEY_S:
			set_viewer_mode(ViewerMode::Scale);
			return;
		case GLFW_KEY_SPACE:
			set_viewer_mode(ViewerMode::View);
			return;
		case GLFW_KEY_T:
			set_viewer_mode(ViewerMode::Translation);
			return;
		case GLFW_KEY_D:
			deformationSpace_->toggle_details();
			meshIsDirty_ |= MeshUpdate::Geometry;
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
	// operator keeps mesh properties that should be released first
	if (deformationSpace_) deformationSpace_.reset();

	// load mesh
	if (mesh_.read(filename))
	{
		strcpy_s(fileNameBuffer_, filename);

		// update scene center and bounds
		BoundingBox bb = mesh_.bounds();
		set_scene((vec3)bb.center(), 0.5 * bb.size());
		meshHandle_.set_scale(vec3(bb.size() * 0.1f));

		// compute face & vertex normals, update face indices
		update_mesh();
		mesh_.update_color_buffer();

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

		filename_ = filename;

		// construct modifier
		deformationSpace_ = std::make_unique<algorithm::Deformation>(mesh_);

		// construct structure for picking
		init_picking();

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
		meshHandle_.set_mouseStartPos(vec2(last_point_2d_[0], last_point_2d_[1]));
		deformationSpace_->reset_scale_origin();
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

std::vector<Vertex> VertexSelectionViewer::pick_vertex(int x, int y, float radius, bool onlyConnected)
{
	std::vector<Vertex> vVector;

	vec3 p;

	if (TrackballViewer::pick(x, y, p))
	{
		SphereQuery query;
		query.center_   = p;
		query.radius_   = radius;
		query.radiusSq_ = radius*radius;
		queryTree_.traverse(query);

		if (!onlyConnected)
		{
			vVector = std::move(query.verticesHit);
		}
		else if(query.verticesHit.size())
		{
			auto points = mesh_.get_vertex_property<Point>("v:point");

			auto marks = mesh_.add_vertex_property<int>("v:tempMark", 0);
			for (Vertex v : query.verticesHit) marks[v] = 1;

			// find closest vertex
			float minDistSq = std::numeric_limits<float>::max();
			Vertex minV;
			for (Vertex v : query.verticesHit)
			{
				const float d = sqrnorm(p - points[v]);
				if (d < minDistSq) 
				{
					minV = v;
					minDistSq = d;
				}
			}

			vVector.push_back(minV);
			marks[minV] = 0;
			size_t cur = 0;
			while (cur < vVector.size())
			{
				Vertex v = vVector[cur];
				marks[v] = 0;

				for (Vertex nv : mesh_.vertices(v))
				{
					if (marks[nv])
					{
						vVector.push_back(nv);
						marks[nv] = 0;
					}
				}
				++cur;
			}
			mesh_.remove_vertex_property(marks);
		}
	}

	return vVector;
}

void VertexSelectionViewer::process_imgui()
{
	ImGui::InputText("", fileNameBuffer_, 512);
	if (ImGui::Button("open"))
	{
		if (load_mesh(fileNameBuffer_)) set_viewer_mode(ViewerMode::View);
	}
	ImGui::SameLine();
	if (ImGui::Button("save"))
	{
		mesh_.write(fileNameBuffer_);
	}

	if (viewerMode_ == ViewerMode::View && ImGui::CollapsingHeader("Selection", ImGuiTreeNodeFlags_DefaultOpen))
	{
		constexpr static const char* VERTEX_DRAW_NAMES[] = { "View [R]", "Clear [E]", "Handle [Q]", "Support [W]" };
		// display possible changes from hotkeys
		currentVertexDrawItem_ = VERTEX_DRAW_NAMES[static_cast<size_t>(vertexDrawingMode_)];
		if (ImGui::BeginCombo("draw mode", currentVertexDrawItem_))
		{
			for (int i = 0; i < 4; ++i)
			{
				const bool isSelected = currentVertexDrawItem_ == VERTEX_DRAW_NAMES[i];
				if (ImGui::Selectable(VERTEX_DRAW_NAMES[i], isSelected))
				{
					currentVertexDrawItem_ = VERTEX_DRAW_NAMES[i];
					vertexDrawingMode_ = static_cast<VertexDrawingMode>(i);
				}
				if (isSelected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		// todo: move this information to a better place?
		if (viewerMode_ != ViewerMode::View)
		{
			ImGui::BulletText("X: Select local x-axis");
			ImGui::BulletText("Y: Select local y-axis");
			ImGui::BulletText("Z: Select local z-axis");
			ImGui::Separator();
			ImGui::Text(("Current ViewMode " + viewerMode_).c_str());
		}
		const BoundingBox bb = mesh_.bounds();
		ImGui::SliderFloat("brush size", &brushSize_, 0.0001, bb.size() * 0.5);
	}
	if (viewerMode_ != ViewerMode::View && ImGui::CollapsingHeader("Operator", ImGuiTreeNodeFlags_DefaultOpen))
	{
		constexpr static const char* MODIFIER_NAMES[] = { "Translate [T]", "Rotate [R]", "Scale [S]"};
		// display possible changes from hotkeys
		currentModifierItem_ = MODIFIER_NAMES[static_cast<size_t>(viewerMode_)-1];
		if (ImGui::BeginCombo("modifier", currentModifierItem_))
		{
			for (int i = 0; i < 3; ++i)
			{
				const bool isSelected = currentModifierItem_ == MODIFIER_NAMES[i];
				if (ImGui::Selectable(MODIFIER_NAMES[i], isSelected))
				{
					currentModifierItem_ = MODIFIER_NAMES[i];
					set_viewer_mode(static_cast<ViewerMode>(i+1));
				}
				if (isSelected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		if (ImGui::SliderInt("Order", &operatorOrder_, 1, 3))
		{
			deformationSpace_->set_order(operatorOrder_);
			meshIsDirty_ |= MeshUpdate::Geometry;
		}
		if (ImGui::Checkbox("area scaling", &useAreaScaling_))
		{
			deformationSpace_->set_area_scaling(useAreaScaling_);
			meshIsDirty_ |= MeshUpdate::Geometry;
		}
		ImGui::Separator();
		ImGui::Text("smoothness");
		if (ImGui::SliderFloat("handle", &smoothnessHandle_, 0.f, 2.f))
		{
			deformationSpace_->set_smoothness_handle(smoothnessHandle_);
			meshIsDirty_ |= MeshUpdate::Geometry;
		}
		if (ImGui::SliderFloat("boundary", &smoothnessBoundary_, 0.f, 2.f))
		{
			deformationSpace_->set_smoothness_boundary(smoothnessBoundary_);
			meshIsDirty_ |= MeshUpdate::Geometry;
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
	vec2 midScreen = compute_screenCoordinates(meshHandle_.origin());


	if ((ypos > 0 || xpos > 0) && mouseMotionNorm > 0)
	{
		mouseMotion = vec2(mouseMotion[0], mouseMotion[1]);
		vec3 movement = meshHandle_.compute_move_vector(projection_matrix_ * modelview_matrix_, width(), height(), mouseMotion);
		deformationSpace_->translate(movement);
		translationPoint_ += movement;
		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ |= MeshUpdate::Geometry;
	}
}

void VertexSelectionViewer::rotationHandle(float xpos, float ypos)
{
	vec2 midScreen = compute_screenCoordinates(meshHandle_.origin());
	vec2 currMousePos = vec2(xpos, ypos) - midScreen;
	vec2 lastPos = vec2(last_point_2d_[0], last_point_2d_[1]) - midScreen;
	vec2 mouseMotion = currMousePos - vec2(last_point_2d_[0], last_point_2d_[1]);
	float mouseMotionNorm = pmp::norm(mouseMotion);

	currMousePos.normalize();
	lastPos.normalize();

	if (mouseMotionNorm > 0)
	{
		float angle = (atan2(lastPos[1], lastPos[0]) - atan2(currMousePos[1], currMousePos[0]));
		angle *= 180.0f / M_PI;

		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		float x = xpos;
		float y = ypos;

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

		vec3 dir3 = vec3(dir[0], dir[1], dir[2]);

		double dotAngle = atan2(norm(cross(dir3, meshHandle_.compute_rotation_vector())), dot(dir3, meshHandle_.compute_rotation_vector()));
		dotAngle *= 180.0f / M_PI;

		if (dotAngle < 90)
			deformationSpace_->rotate(meshHandle_.compute_rotation_vector(), -angle);
		else
			deformationSpace_->rotate(meshHandle_.compute_rotation_vector(), angle);

		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ |= MeshUpdate::Geometry;
	}
}

void VertexSelectionViewer::scaleHandle(float xpos, float ypos)
{
	vec2 midScreen = compute_screenCoordinates(meshHandle_.origin());
	vec2 currMousePos = vec2(xpos, ypos) - midScreen;
	vec2 startPos = meshHandle_.get_mouseStartPos() - midScreen;
	float maxNorm = norm(startPos);

	float mouseMotionNorm = norm(currMousePos) - norm(startPos);
	if (mouseMotionNorm != 0)
	{
		deformationSpace_->scale(norm(currMousePos) / maxNorm);
		last_point_2d_ = ivec2(xpos, ypos);
		meshIsDirty_ |= MeshUpdate::Geometry;
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

bool VertexSelectionViewer::init_modifier()
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

	if (!supportVertices.size() || !handleVertices.size()) return false;

	translationPoint_ = handlePoint / pointIndex;
	normal.normalize();
	translationNormal_ = normal;

	deformationSpace_->set_regions(supportVertices, handleVertices);

	meshHandle_.init_local_coordinate_system(modelview_matrix_, translationNormal_);
	meshIsDirty_ |= MeshUpdate::Geometry;

	return true;
}

void VertexSelectionViewer::init_picking()
{
	queryTree_.clear();

	auto points = mesh_.get_vertex_property<Point>("v:point");
	for (Vertex v : mesh_.vertices())
		queryTree_.insert(points[v], v);
}

void VertexSelectionViewer::draw_on_mesh()
{
	TrackballViewer::pick(pickPosition_);
	double x = 0;
	double y = 0;
	cursor_pos(x, y);
	auto vVector = pick_vertex(x, y, brushSize_, false);

	if (!vVector.empty())
	{
		auto vProp = mesh_.get_vertex_property<Color>("v:col");
		for (auto v : vVector)
			vProp[v] = COLORS[static_cast<size_t>(vertexDrawingMode_)];
		meshIsDirty_ |= MeshUpdate::VertexColor;
	}
}

vec2 VertexSelectionViewer::compute_screenCoordinates(vec3 vec)
{
	vec4 t = projection_matrix_ * modelview_matrix_ * vec4(vec, 1.0f);
	t /= t[3];
	vec2 tVec2 = vec2(t[0], -t[1]);
	tVec2 += vec2(1, 1);
	tVec2 = vec2(tVec2[0] * width() * 0.5f, tVec2[1] * height() * 0.5f);

	return tVec2;
}

void VertexSelectionViewer::set_viewer_mode(ViewerMode mode)
{
	if (mode == viewerMode_) return;

	if (viewerMode_ == ViewerMode::View)
		vertexDrawingMode_ = VertexDrawingMode::None;

	switch (mode)
	{
	case ViewerMode::Rotation:
		meshHandle_.set_rotationMode();
		break;
	case ViewerMode::Scale:
		meshHandle_.set_scaleMode();
		break;
	case ViewerMode::Translation:
		meshHandle_.set_translationMode();
		break;
	case ViewerMode::View:
		deformationSpace_->reset_regions();
		init_picking();
		break;
	}

	viewerMode_ = mode;
}

bool VertexSelectionViewer::SphereQuery::descend(const pmp::vec3& center, double size) const
{
	const double fact = std::sqrt(3) * 2.0;
	const double d = size * fact + radius_;

	return sqrnorm(center - center_) < d * d;
}

void VertexSelectionViewer::SphereQuery::process(const pmp::vec3& key, Vertex v)
{
	if (sqrnorm(key - center_) < radiusSq_)
		verticesHit.push_back(v);
}