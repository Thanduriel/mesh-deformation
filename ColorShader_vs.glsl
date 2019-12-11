#version 330

layout (location=0) in vec4 v_position;
layout (location=1) in vec3 v_normal;
layout (location=2) in vec2 v_tex;
layout (location=3) in vec3 v_color;
out vec3 v2f_normal;
out vec2 v2f_tex;
out vec3 v2f_view;
out vec3 vertex_color;
uniform mat4 modelview_projection_matrix;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;
uniform float point_size;
uniform bool show_texture_layout;

void main()
{
   v2f_normal  = normal_matrix * v_normal;
   v2f_tex     = v_tex;
   vec4 pos    = show_texture_layout ? vec4(v_tex, 0.0, 1.0) : v_position;
   v2f_view    = -(modelview_matrix * pos).xyz;
   gl_PointSize = point_size;
   gl_Position = modelview_projection_matrix * pos;
   vertex_color = v_color;
} ;