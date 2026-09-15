/*    
    Copyright (c) 2020 Patryk Cieslak. All rights reserved.

    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#version 430

layout (points) in;
layout (line_strip, max_vertices = 2) out;

uniform mat4 VP;
uniform float vectorSize;
uniform float velocityMax;
uniform vec3 eyePos;
uniform sampler3D fieldTex;
uniform vec3 fieldBoxMin;
uniform vec3 fieldBoxSize;

vec3 sampleFieldTex(vec3 p)
{
    vec3 tc = (p - fieldBoxMin) / fieldBoxSize;   //world -> [0,1] texcoord
    return texture(fieldTex, tc).xyz;             //trilinear; .w holds |v| if you want it
}

#inject "airVelocityField.glsl"

out GS_OUT
{
    vec4 color;
} gs_out;

float rand(vec2 co)
{
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

vec3 colormap(float value, float limit)
{
    vec3 c;
    float data = value/limit;
    c.r = clamp((data-0.375)*4.0, 0.0, 1.0) - clamp((data-0.875)*4.0, 0.0, 0.5);
    c.g = clamp((data-0.125)*4.0, 0.0, 1.0) - clamp((data-0.625)*4.0, 0.0, 1.0);
    c.b = 0.5 + clamp(data*4.0, 0.0, 0.5) - clamp((data-0.375)*4.0, 0.0, 1.0);
    return c;
}

void main()
{
    vec3 p = gl_in[0].gl_Position.xyz;

    float d = length(eyePos - p);
    if(d > 10.0)
        return;

    vec3 velocity = sampleFieldTex(p);

    float vmag = length(velocity);
    if(vmag > 0.01)
    {
        float opacity = exp(-0.1*d);
        gl_Position = VP * vec4(p, 1.0);
        gs_out.color = vec4(0.0);
        EmitVertex();
        gl_Position = VP * vec4(p + velocity/vmag * vectorSize * vmag, 1.0);
        gs_out.color = vec4(colormap(vmag, velocityMax), opacity);
        EmitVertex();
        EndPrimitive();
    }
}