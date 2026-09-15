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
//  OpenGLCurrentTracers.cpp
//  Stonefish
//

#include "graphics/OpenGLCurrentTracers.h"

#include <cmath>
#include "core/GraphicalSimulationApp.h"
#include "graphics/OpenGLState.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLView.h"
#include "graphics/OpenGLContent.h"
#include "utils/SystemUtil.hpp"

namespace sf
{

OpenGLCurrentTracers::OpenGLCurrentTracers(GLuint numParticles, GLfloat range, GLfloat keepSign)
    : uniformd(0.f, 1.f), normald(0.f, 1.f)
{
    nParticles = numParticles;
    this->range = fabsf(range);
    this->keepSign = (keepSign >= 0.f) ? 1.f : -1.f;
    initialised = false;
    generator = std::mt19937((unsigned int)GetTimeInMicroseconds());

    //Allocate the persistent position buffer (filled lazily by Seed())
    glGenBuffers(1, &posSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(glm::vec4) * nParticles, NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

OpenGLCurrentTracers::~OpenGLCurrentTracers()
{
    if(posSSBO != 0) glDeleteBuffers(1, &posSSBO);
}

void OpenGLCurrentTracers::Seed(glm::vec3 eyePos)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSBO);
    glm::vec4* pos = (glm::vec4*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0,
                        sizeof(glm::vec4) * nParticles,
                        GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    for(GLuint i=0; i<nParticles; ++i)
    {
        GLfloat r = cbrtf(uniformd(generator)) * range;   //cbrt => uniform density in volume
        glm::vec3 dir = glm::normalize(glm::vec3(normald(generator), normald(generator), normald(generator)));
        glm::vec3 p = eyePos + r * dir;
        if(keepSign * p.z < 0.f) p.z = -p.z;              //reflect into the valid half-space
        pos[i] = glm::vec4(p, uniformd(generator) * 4.0f);   //w = random initial age in [0, maxAge]
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    initialised = true;
}

void OpenGLCurrentTracers::Update(OpenGLView* view, GLfloat dt, GLSLShader* advectShader)
{
    const GLint N = 32;
    std::vector<glm::vec4> grid(N*N*N);
    glm::vec3 eye = view->GetEyePosition();
    glm::vec3 boxMin = eye - glm::vec3(10.f);
    glm::vec3 boxSize = glm::vec3(20.f);

    // Synthetic field for debug
    // for(int z=0; z<N; ++z)
    //     for(int y=0; y<N; ++y)
    //     for(int x=0; x<N; ++x)
    //     {
    //         glm::vec3 t  = (glm::vec3(x,y,z) + 0.5f) / (GLfloat)N;  //cell centers
    //         glm::vec3 wp = boxMin + t * boxSize;                    //world pos of this cell
    //         glm::vec3 vel = glm::vec3(0.f, wp.x, 0.f);              //synthetic: vy = world x
    //         grid[x + N*(y + N*z)] = glm::vec4(vel, glm::length(vel));
    //     }
    // UploadField(grid, N, boxMin, boxSize);

    //Verify: read one texel back and compare to the CPU value we wrote
    std::vector<glm::vec4> check(N*N*N);
    glBindTexture(GL_TEXTURE_3D, fieldTex);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, check.data());
    glBindTexture(GL_TEXTURE_3D, 0);
    int idx = 5 + N*(5 + N*5);
    glm::vec3 wp = boxMin + (glm::vec3(5,5,5)+0.5f)/(GLfloat)N * boxSize;
    // cInfo("FIELD texel(5,5,5) gpu.vy=%.3f expected(wp.x)=%.3f", check[idx].y, wp.x);

    // cInfo("Update dt=%.6f", dt);
    glm::vec3 eyePos = view->GetEyePosition();
    if(!initialised)
        Seed(eyePos);

    advectShader->Use();
    advectShader->SetUniform("dt", dt);
    advectShader->SetUniform("numParticles", nParticles);
    advectShader->SetUniform("eyePos", eyePos);
    advectShader->SetUniform("R", range);

    OpenGLState::BindTexture(TEX_POSTPROCESS5, GL_TEXTURE_3D, fieldTex);
    advectShader->SetUniform("fieldTex", TEX_POSTPROCESS5);
    advectShader->SetUniform("fieldBoxMin", fieldBoxMin);
    advectShader->SetUniform("fieldBoxSize", fieldBoxSize);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_PARTICLE_POS, posSSBO);
    glDispatchCompute((GLuint)ceil(nParticles/256.0), 1, 1);
    OpenGLState::UseProgram(0);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    OpenGLState::UnbindTexture(TEX_POSTPROCESS5); 

    // glm::vec4 c[2];
    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSBO);
    // glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(c), c);
    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    // cInfo("POS p0=(%.3f,%.3f,%.3f) p1=(%.3f,%.3f,%.3f)", c[0].x,c[0].y,c[0].z, c[1].x,c[1].y,c[1].z);
}

void OpenGLCurrentTracers::UploadField(const std::vector<glm::vec4>& grid, GLint N,
                                       glm::vec3 boxMin, glm::vec3 boxSize)
{
    fieldBoxMin = boxMin;
    fieldBoxSize = boxSize;

    glBindTexture(GL_TEXTURE_3D, (fieldTex == 0) ? (glGenTextures(1, &fieldTex), fieldTex) : fieldTex);

    if(gridN != N)   //(re)allocate storage when size changes or on first use
    {
        gridN = N;
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, N, N, N, 0, GL_RGBA, GL_FLOAT, grid.data());
    }
    else             //same size -> cheaper sub-update
    {
        glTexSubImage3D(GL_TEXTURE_3D, 0, 0,0,0, N, N, N, GL_RGBA, GL_FLOAT, grid.data());
    }
    glBindTexture(GL_TEXTURE_3D, 0);
}

void OpenGLCurrentTracers::Draw(OpenGLView* view, GLfloat velocityMax, GLSLShader* renderShader)
{
    // cInfo("Tracers::Draw ENTER, preexisting glErr=0x%x", glGetError());
    if(!initialised)
        return;

    //--- Waterline cull: skip drawing when the limited frustum is entirely in the
    //    invalid half-space (e.g. fully underwater for an air field). ---
    const GLfloat LARGE = 1.0e9f;
    glm::vec3 eyePos = view->GetEyePosition();
    glm::mat4 V = view->GetViewMatrix();
    glm::mat4 invP = glm::inverse(view->GetProjectionMatrix());
    glm::mat4 invV = glm::inverse(V);

    glm::vec4 proj[4];
    proj[0] = invP * glm::vec4(-1.f, -1.f, -1.f, 1.f);
    proj[1] = invP * glm::vec4( 1.f, -1.f, -1.f, 1.f);
    proj[2] = invP * glm::vec4( 1.f,  1.f, -1.f, 1.f);
    proj[3] = invP * glm::vec4(-1.f,  1.f, -1.f, 1.f);

    GLfloat scaling = range / view->GetNearClip();
    glm::vec3 corner[5];
    corner[0] = eyePos;
    for(int i=0; i<4; ++i)
    {
        glm::vec3 farPt = glm::vec3(invV * (proj[i]/proj[i].w));
        corner[i+1] = eyePos + (farPt - eyePos) * scaling;
    }

    GLfloat zMin =  LARGE;
    GLfloat zMax = -LARGE;
    for(int i=0; i<5; ++i)
    {
        if(corner[i].z < zMin) zMin = corner[i].z;
        if(corner[i].z > zMax) zMax = corner[i].z;
    }
    //Largest "valid-ness" anywhere in the box; if <= 0 the whole box is invalid.
    GLfloat maxValid = (keepSign > 0.f) ? zMax : -zMin;
    // cInfo("Tracers::Draw zMin=%.2f zMax=%.2f maxValid=%.2f", zMin, zMax, maxValid);
    if(maxValid <= 0.f)
        return;

    //--- Draw the arrows from the particle buffer ---
    renderShader->Use();
    // cInfo("after Use glErr=0x%x", glGetError());
    renderShader->SetUniform("vectorSize", 0.2f);
    renderShader->SetUniform("velocityMax", velocityMax);
    renderShader->SetUniform("VP", view->GetProjectionMatrix() * V);
    renderShader->SetUniform("eyePos", eyePos);
    
    OpenGLState::BindTexture(TEX_POSTPROCESS5, GL_TEXTURE_3D, fieldTex);
    renderShader->SetUniform("fieldTex", TEX_POSTPROCESS5);
    renderShader->SetUniform("fieldBoxMin", fieldBoxMin);
    renderShader->SetUniform("fieldBoxSize", fieldBoxSize);

    // cInfo("after uniforms glErr=0x%x", glGetError());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_PARTICLE_POS, posSSBO);
    OpenGLState::EnableBlend();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    ((GraphicalSimulationApp*)SimulationApp::getApp())->getGLPipeline()->getContent()->BindBaseVertexArray();
    glDrawArrays(GL_POINTS, 0, (GLsizei)nParticles);

    OpenGLState::BindVertexArray(0);
    OpenGLState::UseProgram(0);
    OpenGLState::DisableBlend();
    OpenGLState::UnbindTexture(TEX_POSTPROCESS5);
}

}
