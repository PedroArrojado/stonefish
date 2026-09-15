/*    
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

//
//  OpenGLCurrentTracers.h
//  Stonefish
//
//  Shared machinery for advected velocity-field tracer particles ("lines as
//  particles"). Used by both OpenGLAtmosphere (air currents) and OpenGLOcean
//  (water currents). The class owns only the particle position buffer and the
//  generic GL plumbing (seed / dispatch / draw). It is deliberately shader-
//  agnostic: the caller passes in the compute shader (advection) and the render
//  shader (arrow glyph), so the field physics stays per-domain and can diverge.
//

#ifndef __Stonefish_OpenGLCurrentTracers__
#define __Stonefish_OpenGLCurrentTracers__

#include <random>
#include "graphics/OpenGLDataStructs.h"
#include <vector>

namespace sf
{
    class GLSLShader;
    class OpenGLView;

    //! A class implementing advected tracer particles that visualize a velocity field.
    class OpenGLCurrentTracers
    {
    public:
        //! A constructor.
        /*!
         \param numParticles the number of tracer particles
         \param range the radius around the camera within which particles live [m]
         \param keepSign which half-space particles occupy: +1 keeps z>0 (underwater),
                          -1 keeps z<0 (air)
         */
        OpenGLCurrentTracers(GLuint numParticles, GLfloat range, GLfloat keepSign);

        //! A destructor.
        ~OpenGLCurrentTracers();

        //! Advance the particles by one step using the given compute shader.
        /*!
         \param view a pointer to the active view (provides the camera position)
         \param dt the time step [s]
         \param advectShader the compute shader that samples the field and moves particles
         */
        void Update(OpenGLView* view, GLfloat dt, GLSLShader* advectShader);

        //! Draw the particles as field arrows using the given render shader.
        /*!
         \param view a pointer to the active view
         \param velocityMax velocity corresponding to the color/length display limit
         \param renderShader the vectorfield (vert+geom+frag) shader drawing the arrows
         */
        void Draw(OpenGLView* view, GLfloat velocityMax, GLSLShader* renderShader);

        void UploadField(const std::vector<glm::vec4>& grid, GLint N,
                         glm::vec3 boxMin, glm::vec3 boxSize);
        
        GLuint fieldTex;          //3D texture holding sampled field velocity (xyz) + |v| (w)
        GLint  gridN;             //texture resolution per axis
        glm::vec3 fieldBoxMin;    //world-space min corner of the sampled box
        glm::vec3 fieldBoxSize;   //world-space size of the sampled box

    private:
        void Seed(glm::vec3 eyePos);

        GLuint posSSBO;          //vec4 per particle: xyz = position, w = free (size/age)
        GLuint nParticles;
        GLfloat range;
        GLfloat keepSign;        //+1 => keep z>0 (ocean), -1 => keep z<0 (air)
        bool initialised;

        std::mt19937 generator;
        std::uniform_real_distribution<GLfloat> uniformd;
        std::normal_distribution<GLfloat> normald;
    };
}

#endif
