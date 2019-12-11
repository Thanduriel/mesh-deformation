#version 330
precision mediump float;

in vec3  v2f_normal;
in vec2  v2f_tex;
in vec3  v2f_view;
in vec3  vertex_color;

uniform bool   use_lighting;
uniform bool   use_texture;
uniform bool   use_srgb;
uniform vec3   front_color;
uniform vec3   back_color;
uniform float  ambient;
uniform float  diffuse;
uniform float  specular;
uniform float  shininess;
uniform float  alpha;
uniform vec3   light1;
uniform vec3   light2;

uniform sampler2D mytexture;

out vec4 f_color;

void main()
{
    vec3 color = gl_FrontFacing ? front_color : back_color;
    vec3 rgb;

    if (use_lighting)
    {
       vec3 L1 = normalize(light1);
       vec3 L2 = normalize(light2);
       vec3 N  = normalize(v2f_normal);
       vec3 V  = normalize(v2f_view);
       
//       if (!gl_FrontFacing) N = -N; // gl_FrontFacing does not work on Mario's new MacBook
       if (dot(N,V)<0.0) N = -N;
       
       vec3  R;
       float NL, RV;
       
       rgb = ambient * 0.1 * color;
       
       NL = dot(N, L1);
       if (NL > 0.0)
       {
           rgb += diffuse * NL * color;
           R  = normalize(-reflect(L1, N));
           RV = dot(R, V);
           if (RV > 0.0) 
           {
               rgb += vec3( specular * pow(RV, shininess) );
           }
       }
       
       NL = dot(N, L2);
       if (NL > 0.0)
       {
            rgb += diffuse * NL * color;
            R  = normalize(-reflect(L2, N));
            RV = dot(R, V);
            if (RV > 0.0) 
            {
                rgb += vec3( specular * pow(RV, shininess) );
            }
        }
    }
   
    // do not use lighting
    else
    {
        rgb = color;
    }

    if (use_texture) rgb *= texture(mytexture, v2f_tex).xyz;
    if (use_srgb)    rgb  = pow(clamp(rgb, 0.0, 1.0), vec3(0.45));

    rgb *= vertex_color;
   
    f_color = vec4(rgb, alpha);
};