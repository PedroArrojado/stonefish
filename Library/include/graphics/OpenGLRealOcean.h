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
//  OpenGLRealOcean.h
//  Stonefish
//
//  Created by Patryk Cieslak on 10/05/2020.
//  Copyright (c) 2020-2024 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_OpenGLRealOcean__
#define __Stonefish_OpenGLRealOcean__

#include "graphics/OpenGLOcean.h"
#include <SDL2/SDL_mutex.h>
#include <functional>

namespace sf
{
    //! A structure hold the quad-tree information for each camera.
    struct OceanQT
    {
        GLuint patchSSBO[4];
        GLuint patchAC;
        GLuint patchDEI;
        GLuint patchDI;
        GLint pingpong;
    };

    //! A class implementing reallistic deformed ocean in OpenGL.
    class OpenGLRealOcean : public OpenGLOcean
    {
    public:
        //! A constructor.
        /*!
         \param size the size of the ocean surface mesh [m]
         \param state the state of the ocean, if >0 the ocean is rendered with geometric waves otherwise as a plane with wave texture
         
         NEW:param based construction
         \param oceanType the type of ocean construction, either "params" or "sea_state" (original constructor)
         \param eckvWindSpeed the speed of the wind [m/s] U10 in ECKV see paper
         \param eckvDirection the direction of the wind [rad] (half plane mask direction) see paper
         \param eckvAge the age of the wind [s] U10 in ECKV see paper
         
         Original mutex pass
         \param hydrodynamics a pointer to a mutex
         */
        OpenGLRealOcean(GLfloat size, GLfloat state, std::string oceanType,
            GLfloat eckvWindSpeed, GLfloat eckvDirection, GLfloat eckvAge, 
            SDL_mutex* hydrodynamics);
        
        //! A destructor.
        ~OpenGLRealOcean();
        
        // NEW: For mutex access
        SDL_mutex* getHydroMutex();
        
        /*! 
         NEW: Setup a callback function to be called when ocean parameters are updated.
         This is used to apply the pending ocean update in the next render call.
         \param cb a callback function to be called when ocean parameters are updated
        */
        void setUpdateCallback(std::function<void()> cb) { updateCallback_ = cb;}

        //! A method that simulates wave propagation.
		/*!
		 \param dt time since last update
		 */
        void Simulate(GLfloat dt) override;
         
        //! A method that resets the quad tree.
        /*!
         \param view a pointer to the active view
         */
        void ResetSurface(OpenGLView* view);
        
        //! A method that updates the wave mesh.
        /*!
         \param view a pointer to the active view
         */
        void UpdateSurface(OpenGLView* view) override;

        //! A method that draws the surface of the ocean.
        /*!
         \param view a pointer to the active view
         */
        void DrawSurface(OpenGLView* view) override;

        //! A method that draws the surface of the ocean as thermal image.
        /*!
         \param view a pointer to the active view
         */
        void DrawSurfaceTemperature(OpenGLView* view) override;
        
        //! A method that draws the surface of the ocean, seen from underwater.
        /*!
         \param view a pointer to the active view
         */
        void DrawBacksurface(OpenGLView* view) override;
        
        //! A method that generates the stencil mask.
        /*!
         \param view a pointer to the active view
         */
        void DrawUnderwaterMask(OpenGLView* view) override;
                
        //! A method to get wave height at a specified coordinate.
        /*!
         \param x the x coordinate in world frame [m]
         \param y the y coordinate in world frame [m]
         \return wave height [m]
         */
        GLfloat ComputeWaveHeight(GLfloat x, GLfloat y) override;

        //! A method to get wave height at a batch of coordinates.
        /*!
         \param pts a vector of points in world frame [m]
         \return a vector of wave heights [m]
         */
        virtual std::vector<float> ComputeWaveHeightMap(const std::vector<glm::vec3>& pts) override;

        //! A method do enable wireframe rendering.
        /*!
         \param enabled a flag to indicating if wireframe should be enabled
         */
        void setWireframe(bool enabled);
        
    private:
        void InitializeSimulation() override;
        GLfloat ComputeInterpolatedWaveData(GLfloat x, GLfloat y, GLuint channel);

        GLuint vao;
        GLuint oceanBuffers[2];
        GLuint fftPBO;
        std::map<OpenGLView*, OceanQT> oceanTrees; 
        SDL_mutex* hydroMutex;
        GLfloat* fftData;
        GLint qtGridTessFactor;
        GLint qtGPUTessFactor;
        GLint qtPatchIndexCount;
        bool wireframe;
        
        // NEW: Callback function to be called when ocean parameters are updated
        std::function<void()> updateCallback_;
    };
}

#endif
