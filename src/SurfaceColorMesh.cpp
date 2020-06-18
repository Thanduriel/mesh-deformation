#include "SurfaceColorMesh.hpp"
#include <pmp/algorithms/SurfaceNormals.h>
#include <filesystem>

using namespace pmp;

Shader* SurfaceColorMesh::color_shader_ = nullptr;
int SurfaceColorMesh::shader_ref_count_ = 0;

SurfaceColorMesh::SurfaceColorMesh()
{
	// initialize GL buffers to zero
	vertex_array_object_ = 0;
	vertex_buffer_ = 0;
	vertex_color_buffer_ = 0;
	normal_buffer_ = 0;
	tex_coord_buffer_ = 0;
	edge_buffer_ = 0;
	feature_buffer_ = 0;

	// initialize buffer sizes
	n_vertices_ = 0;
	n_edges_ = 0;
	n_triangles_ = 0;
	n_features_ = 0;
	have_texcoords_ = false;

	// material parameters
	front_color_ = vec3(0.6, 0.6, 0.6);
	back_color_ = vec3(0.5, 0.0, 0.0);
	ambient_ = 0.1;
	diffuse_ = 0.8;
	specular_ = 0.6;
	shininess_ = 100.0;
	alpha_ = 1.0;
	srgb_ = false;

	++shader_ref_count_;
}

SurfaceColorMesh::~SurfaceColorMesh()
{
	// delete OpenGL buffers
	glDeleteBuffers(1, &vertex_buffer_);
	glDeleteBuffers(1, &normal_buffer_);
	glDeleteBuffers(1, &tex_coord_buffer_);
	glDeleteBuffers(1, &edge_buffer_);
	glDeleteBuffers(1, &feature_buffer_);
	glDeleteBuffers(1, &vertex_color_buffer_);
	glDeleteVertexArrays(1, &vertex_array_object_);

	--shader_ref_count_;
	if (!shader_ref_count_ && color_shader_)
	{
		delete color_shader_;
		color_shader_ = nullptr;
	}
}

void SurfaceColorMesh::draw(const mat4& projection_matrix, const mat4& modelview_matrix, const std::string& draw_mode)
{
	// did we generate buffers already?
	if (!vertex_array_object_)
	{
		update_opengl_buffers();
	}

	// load shader?
	if (!color_shader_)
	{
		color_shader_ = new Shader();
		
		// look in current directory
		if (std::filesystem::exists("shaders/ColorShader_vs.glsl"))
		{
			if (!color_shader_->load("shaders/ColorShader_vs.glsl", "shaders/ColorShader_fs.glsl"))
				exit(1);
		} 
		else if (std::filesystem::exists("../shaders/ColorShader_vs.glsl"))// try folder above
		{
			if (!color_shader_->load("../shaders/ColorShader_vs.glsl", "../shaders/ColorShader_fs.glsl"))
				exit(1);
		}
		else
		{
			std::cerr << "Could not find shaders in 'shaders/' or '../shaders/'. Looking for 'ColorShader_vs.glsl' and 'ColorShader_fs.glsl' !";
			exit(1);
		}
	}

	// empty mesh?
	if (is_empty())
		return;

	// setup matrices
	mat4 mv_matrix = modelview_matrix;
	mat4 mvp_matrix = projection_matrix * modelview_matrix;
	mat3 n_matrix = inverse(transpose(linear_part(mv_matrix)));

	// setup shader
	color_shader_->use();
	color_shader_->set_uniform("modelview_projection_matrix", mvp_matrix);
	color_shader_->set_uniform("modelview_matrix", mv_matrix);
	color_shader_->set_uniform("normal_matrix", n_matrix);
	color_shader_->set_uniform("point_size", 5.0f);
	color_shader_->set_uniform("light1", vec3(1.0, 1.0, 1.0));
	color_shader_->set_uniform("light2", vec3(-1.0, 1.0, 1.0));
	color_shader_->set_uniform("front_color", front_color_);
	color_shader_->set_uniform("back_color", back_color_);
	color_shader_->set_uniform("ambient", ambient_);
	color_shader_->set_uniform("diffuse", diffuse_);
	color_shader_->set_uniform("specular", specular_);
	color_shader_->set_uniform("shininess", shininess_);
	color_shader_->set_uniform("alpha", alpha_);
	color_shader_->set_uniform("use_lighting", true);
	color_shader_->set_uniform("use_texture", false);
	color_shader_->set_uniform("use_srgb", false);
	color_shader_->set_uniform("show_texture_layout", false);

	glBindVertexArray(vertex_array_object_);

	if (draw_mode == "Points")
	{
#ifndef __EMSCRIPTEN__
		glEnable(GL_PROGRAM_POINT_SIZE);
#endif
		glDrawArrays(GL_POINTS, 0, n_vertices_);
	}
	else if (draw_mode == "Smooth Shading")
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		if (n_faces())
		{
			glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
		}
	}
	else if (draw_mode == "Wireframe")
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		if (n_faces())
		{
			glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
		}
	}

	glBindVertexArray(0);
	glCheckError();
}

void SurfaceColorMesh::update_opengl_buffers()
{
	// are buffers already initialized?
	if (!vertex_array_object_)
	{
		glGenVertexArrays(1, &vertex_array_object_);
		glBindVertexArray(vertex_array_object_);
		glGenBuffers(1, &vertex_buffer_);
		glGenBuffers(1, &normal_buffer_);
		glGenBuffers(1, &tex_coord_buffer_);
		glGenBuffers(1, &edge_buffer_);
		glGenBuffers(1, &feature_buffer_);
		glGenBuffers(1, &vertex_color_buffer_);
	}

	// activate VAO
	glBindVertexArray(vertex_array_object_);

	// get vertex properties
	auto vpos = get_vertex_property<Point>("v:point");
	auto vtex = get_vertex_property<TexCoord>("v:tex");
	auto htex = get_halfedge_property<TexCoord>("h:tex");

	// index array for remapping vertex indices during duplication
	auto vertex_indices = add_vertex_property<size_t>("v:index");

	// produce arrays of points, normals, and texcoords
	// (duplicate vertices to allow for flat shading)
	std::vector<vec3> positionArray;
	std::vector<vec3> normalArray;
	std::vector<vec2> texArray;

	// we have a mesh: fill arrays by looping over faces
	if (n_faces())
	{
		// reserve memory
		positionArray.reserve(3 * n_faces());
		normalArray.reserve(3 * n_faces());
		bufferVertices_.clear();
		bufferVertices_.reserve(3 * n_faces());
		if (htex || vtex)
			texArray.reserve(3 * n_faces());

		// precompute normals for easy cases
		VertexProperty<Normal> vnormals;
		vnormals = add_vertex_property<Normal>("gl:vnormal");
		for (auto v : vertices())
			vnormals[v] = SurfaceNormals::compute_vertex_normal(*this, v);

		// data per face (for all corners)
		std::vector<Halfedge> cornerHalfedges;
		std::vector<Vertex> cornerVertices;
		std::vector<vec3> cornerNormals;

		size_t vidx(0);

		// loop over all faces
		for (auto f : faces())
		{
			// collect corner positions and normals
			cornerHalfedges.clear();
			cornerVertices.clear();
			cornerNormals.clear();
			Vertex v;
			Normal n;

			for (auto h : halfedges(f))
			{
				cornerHalfedges.push_back(h);

				v = to_vertex(h);
				cornerVertices.push_back(v);
				n = vnormals[v];
				cornerNormals.push_back((vec3)n);
			}
			assert(cornerVertices.size() >= 3);

			// tessellate face into triangles
			int i0, i1, i2, nc = cornerVertices.size();
			for (i0 = 0, i1 = 1, i2 = 2; i2 < nc; ++i1, ++i2)
			{
				positionArray.push_back((vec3)vpos[cornerVertices[i0]]);
				positionArray.push_back((vec3)vpos[cornerVertices[i1]]);
				positionArray.push_back((vec3)vpos[cornerVertices[i2]]);

				normalArray.push_back((vec3)cornerNormals[i0]);
				normalArray.push_back((vec3)cornerNormals[i1]);
				normalArray.push_back((vec3)cornerNormals[i2]);

				bufferVertices_.push_back(cornerVertices[i0]);
				bufferVertices_.push_back(cornerVertices[i1]);
				bufferVertices_.push_back(cornerVertices[i2]);

				if (htex)
				{
					texArray.push_back((vec2)htex[cornerHalfedges[i0]]);
					texArray.push_back((vec2)htex[cornerHalfedges[i1]]);
					texArray.push_back((vec2)htex[cornerHalfedges[i2]]);
				}
				else if (vtex)
				{
					texArray.push_back((vec2)vtex[cornerVertices[i0]]);
					texArray.push_back((vec2)vtex[cornerVertices[i1]]);
					texArray.push_back((vec2)vtex[cornerVertices[i2]]);
				}

				vertex_indices[cornerVertices[i0]] = vidx++;
				vertex_indices[cornerVertices[i1]] = vidx++;
				vertex_indices[cornerVertices[i2]] = vidx++;
			}
		}

		// clean up
		if (vnormals)
			remove_vertex_property(vnormals);
	}

	// upload vertices
	if (!positionArray.empty())
	{
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
		glBufferData(GL_ARRAY_BUFFER, positionArray.size() * 3 * sizeof(float),
			positionArray.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
		glEnableVertexAttribArray(0);
		n_vertices_ = positionArray.size();
	}
	else
		n_vertices_ = 0;

	// upload normals
	if (!normalArray.empty())
	{
		glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
		glBufferData(GL_ARRAY_BUFFER, normalArray.size() * 3 * sizeof(float),
			normalArray.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
		glEnableVertexAttribArray(1);
	}

	// upload texture coordinates
	if (!texArray.empty())
	{
		glBindBuffer(GL_ARRAY_BUFFER, tex_coord_buffer_);
		glBufferData(GL_ARRAY_BUFFER, texArray.size() * 2 * sizeof(float),
			texArray.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
		glEnableVertexAttribArray(2);
		have_texcoords_ = true;
	}
	else
		have_texcoords_ = false;

	// edge indices
	if (n_edges())
	{
		std::vector<unsigned int> edgeArray;
		edgeArray.reserve(n_edges());
		for (auto e : edges())
		{
			edgeArray.push_back(vertex_indices[vertex(e, 0)]);
			edgeArray.push_back(vertex_indices[vertex(e, 1)]);
		}
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_buffer_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			edgeArray.size() * sizeof(unsigned int), edgeArray.data(),
			GL_STATIC_DRAW);
		n_edges_ = edgeArray.size();
	}
	else
		n_edges_ = 0;

	// feature edges
	auto efeature = get_edge_property<bool>("e:feature");
	if (efeature)
	{
		std::vector<unsigned int> features;

		for (auto e : edges())
		{
			if (efeature[e])
			{
				features.push_back(vertex_indices[vertex(e, 0)]);
				features.push_back(vertex_indices[vertex(e, 1)]);
			}
		}

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, feature_buffer_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			features.size() * sizeof(unsigned int), features.data(),
			GL_STATIC_DRAW);
		n_features_ = features.size();
	}
	else
		n_features_ = 0;

	// unbind vertex arry
	glBindVertexArray(0);

	// remove vertex index property again
	remove_vertex_property(vertex_indices);
}

void SurfaceColorMesh::update_color_buffer()
{
	auto vcol = vertex_property<Color>("v:col", Color(1, 0, 0));

	std::vector<vec3> colorArray;

	if (n_faces())
	{
		colorArray.reserve(3 * n_faces());
		for(Vertex v : bufferVertices_)
			colorArray.push_back((vec3)vcol[v]);

		glBindVertexArray(vertex_array_object_);

		glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_);
		glBufferData(GL_ARRAY_BUFFER, colorArray.size() * 3 * sizeof(float),
			colorArray.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
		glEnableVertexAttribArray(3);

		glBindVertexArray(0);
	}
}
