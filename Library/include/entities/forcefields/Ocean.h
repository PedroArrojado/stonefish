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
//  Ocean.h
//  Stonefish
//
//  Created by Patryk Cieslak on 19/10/17.
//  Copyright (c) 2017-2025 Patryk Cieslak. All rights reserved.
//

#pragma once

#include <SDL2/SDL_mutex.h>
#include "core/MaterialManager.h"
#include "entities/ForcefieldEntity.h"
#include "graphics/OpenGLOcean.h"
#include <atomic>
#include <mutex>

namespace sf
{
    //! A structure holding the settings of the hydrodynamics computation.
    struct HydrodynamicsSettings
    {
        bool dampingForces;
        bool reallisticBuoyancy;
    };
    
    class VelocityField;
    class Actuator;
    
    //! A class implementing an ocean.
    class Ocean : public ForcefieldEntity
    {
    public:
        //! A constructor.
        /*!
         \param uniqueName  a name for the ocean
         \param waves       the state of the ocean (waves enabled when >0)
         \param l           a pointer to the liquid that is filling the ocean (normally water)
         \param oceanType   NEW: a string defining ocean construction type, either "sea_state" for legacy sea state, or "params" for new parameters.
         \param windSpeed   NEW: the speed of the wind [m/s] U10 in ECKV see paper
         \param direction   NEW: the direction of the wind [rad] (half plane mask direction) see paper
         \param age         NEW: the inverse age of the ocean [s] \Omega in ECKV [0.84,5.0] see paper
         */
        Ocean(std::string uniqueName, Scalar waves, Fluid l, 
            std::string oceanType,
            Scalar windSpeed, Scalar direction, Scalar age);
        
        //! A destructor.
        ~Ocean();

        //! A method to set the environmental conditions.
        /*!
         \param waterTemp the temperature of the water at the surface [degC]
        */
        void SetConditions(Scalar waterTemp);

        //! A method used to add a velocity field to the ocean.
        /*!
         \param field a pointer to a velocity field object
         */
        void AddVelocityField(VelocityField* field);
        
        //! A method running the hydrodynamics computation.
        /*!
         \param world a pointer to the dynamics world
         \param co a pointer to the collision object
         \param recompute a flag deciding if hydrodynamic forces need to be recomputed
         */
        void ApplyFluidForces(btDynamicsWorld* world, btCollisionObject* co, bool recompute);
        
        //! A method returning the water velocity.
        /*!
         \param point the point in the ocean where the velocity should be measured [m]
         \return fluid velocity at specified point [m/s]
         */
        Vector3 GetFluidVelocity(const Vector3& point) const;
        glm::vec3 GetFluidVelocity(const glm::vec3& point) const;
        
        //! A method checking if a point is inside fluid
        /*!
         \param point the position of a point to be checked [m]
         \return is the point inside fluid?
         */
        bool IsInsideFluid(const Vector3& point);

        //! NEW: A method checking if a batch of points is inside fluid
        /*!
         \param point the positions of points to be checked [m]
         \return are the points inside fluid?
         */
        std::vector<char> IsInsideFluidMap(const std::vector<Vector3>& points);

        
        //! A method returning the hydrostatic pressure of the fluid at the specified point.
        /*!
         \param point the position of the measurement point [m]
         \return the hydrostatic pressure [Pa]
         */
        Scalar GetPressure(const Vector3& point);
        
        //! A method returning the depth of the ocean at the specified point.
        /*!
         \param point the position of the measurement point [m]
         \return the distance from the point to the surfacer of fluid [m]
         */
        Scalar GetDepth(const Vector3& point);
        GLfloat GetDepth(const glm::vec3& point);

        //! NEW: A method returning the depth of the ocean at a batch of specified points 
        //! (to avoid lock/unlock with update ocean data implementation).
        /*!
         \param points the positions of the measurement points [m]
         \return the distance from the points to the surface of fluid [m]
         */
        std::vector<float> GetDepthMap(const std::vector<glm::vec3>& points);

        
        //! A method to enable all defined currents.
        void EnableCurrents();
        
        //! A method to disable all defined currents.
        void DisableCurrents();

        //! A method updating the currents data in the OpenGL ocean.
        void UpdateCurrentsData();

        //! NEW: A method updating the currents data in the OpenGL ocean. Ocean Updates are applied in the next render call.
        /*!
         \param windSpeed   NEW: the speed of the wind [m/s] U10 in ECKV see paper
         \param direction   NEW: the direction of the wind [rad] (half plane mask direction) see paper
         \param age         NEW: the inverse age of the ocean [s] \Omega in ECKV [0.84,5.0] see paper
        */
        bool UpdateOceanData(Scalar windSpeed, Scalar direction, Scalar age);

        //! NEW: A method applying the pending ocean update in the next render call.
        void ApplyPendingOceanUpdate();
        
        //! A method used to setup the properties of the water.
        /*!
         \param jerlov the type of water according to Jerlov (I-9C) <0,1>
         */
        void setWaterType(Scalar jerlov);

        //! A method to enable rendering of suspended particles (underwater snow).
        void setParticles(bool enabled);

        //! A method informing if the particles are enabled.
        bool hasParticles() const;

        //! A method returning the type of the water.
        Scalar getWaterType() const;
          
        //! A method informing if the ocean waves are simulated.
        bool hasWaves() const;
        
        //! A method returning a pointer to the fluid filling the ocean.
        Fluid getLiquid() const;
        
        //! A method returning a pointer to the OpenGL object implementing the ocean.
        OpenGLOcean* getOpenGLOcean();

        //! A method giving direct access to the defined velocity fields.
        /*!
         \param index an id of the current
         \return pointer to the current object
         */
        VelocityField* getVelocityField(size_t index);

        //! NEW: A method returning a pointer to all velocity fields.
        std::vector<VelocityField*> getVelocityFields();

        //! A method returning the type of the force field.
        ForcefieldType getForcefieldType();
        
        //! A method initializing the rendering of the ocean.
        /*!
         \param hydrodynamics a pointer to a mutex
         */
        void InitGraphics(SDL_mutex* hydrodynamics);
        
        //! A method implementing the rendering of the force field.
        std::vector<Renderable> Render();

        //! A method implementing the rendering of the ocean force field.
        std::vector<Renderable> Render(const std::vector<Actuator*>& act);
        
    private:
        Fluid liquid;
        std::vector<VelocityField*> currents;
        OpenGLOcean* glOcean;
        OceanCurrentsUBO glOceanCurrentsUBOData;
        Scalar depth;
        Scalar waterType;
        Scalar salinity;
        Scalar oceanState;
        
        // NEW: Ocean type and parameters
        std::string oceanType_;
        Scalar eckvWindSpeed;
        Scalar eckvDirection;
        Scalar eckvAge;

        bool currentsEnabled;
        Renderable wavesDebug;

        // NEW: Mutex for thread safety when updating ocean data
        std::atomic<bool> pendingOceanUpdate_{false};
        SDL_mutex* hydroMutex_{nullptr};
    };
}
