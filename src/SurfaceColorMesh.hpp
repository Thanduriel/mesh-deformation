//=============================================================================
// Based on SurfaceMeshGL.
// Copyright (C) 2011-2019 The pmp-library developers
//=============================================================================
#pragma once

#define GLEW_STATIC
#include <pmp/visualization/GL.h>
#include <pmp/visualization/Shader.h>
#include <pmp/MatVec.h>
#include <pmp/SurfaceMesh.h>
#include <memory>

class SurfaceColorMesh : public pmp::SurfaceMesh
{
public:
	SurfaceColorMesh();

	~SurfaceColorMesh();

	//! get front color
	const pmp::vec3& front_color() const { return front_color_; }
	//! set front color
	void set_front_color(const pmp::vec3& color) { front_color_ = color; }

	//! get back color
	const pmp::vec3& back_color() const { return back_color_; }
	//! set back color
	void set_back_color(const pmp::vec3& color) { back_color_ = color; }

	//! get ambient reflection coefficient
	float ambient() const { return ambient_; }
	//! set ambient reflection coefficient
	void set_ambient(float a) { ambient_ = a; }

	//! get diffuse reflection coefficient
	float diffuse() const { return diffuse_; }
	//! set diffuse reflection coefficient
	void set_diffuse(float d) { diffuse_ = d; }

	//! get specular reflection coefficient
	float specular() const { return specular_; }
	//! set specular reflection coefficient
	void set_specular(float s) { specular_ = s; }

	//! get specular shininess coefficient
	float shininess() const { return shininess_; }
	//! set specular shininess coefficient
	void set_shininess(float s) { shininess_ = s; }

	//! get alpha value for transparent rendering
	float alpha() const { return alpha_; }
	//! set alpha value for transparent rendering
	void set_alpha(float a) { alpha_ = a; }

	//! draw the mesh
	void draw(const pmp::mat4& projection_matrix, const pmp::mat4& modelview_matrix,
		const std::string& draw_mode);

	//! update all opengl buffers for efficient core profile rendering
	void update_opengl_buffers();

	// update only the color vertex buffer
	void update_color_buffer();
private:
	//! OpenGL buffers
	GLuint vertex_array_object_;
	GLuint vertex_buffer_;
	GLuint vertex_color_buffer_;
	GLuint normal_buffer_;
	GLuint tex_coord_buffer_;
	GLuint edge_buffer_;
	GLuint feature_buffer_;

	//! buffer sizes
	GLsizei n_vertices_;
	GLsizei n_edges_;
	GLsizei n_triangles_;
	GLsizei n_features_;

	bool have_texcoords_;

	//! shaders
	// SurfaceColorMesh instances have shared ownership of the shader via reference counting.
	static pmp::Shader* color_shader_;
	static int shader_ref_count_;

	//! material properties
	pmp::vec3 front_color_, back_color_;
	float ambient_, diffuse_, specular_, shininess_, alpha_;
	bool srgb_;
	
	// assignment of vertices in the vertex buffer for faster color updates
	std::vector<pmp::Vertex> bufferVertices_;
};

//=============================================================================
//! @}
//=============================================================================
