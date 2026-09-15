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
//  Ocean.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 19/10/17.
//  Copyright(c) 2017-2025 Patryk Cieslak. All rights reserved.
//

#include "entities/forcefields/Ocean.h"

#include <algorithm>
#include "utils/SystemUtil.hpp"
#include "entities/forcefields/VelocityField.h"
#include "entities/SolidEntity.h"
#include "entities/CableEntity.h"
#include "graphics/OpenGLFlatOcean.h"
#include "graphics/OpenGLRealOcean.h"
#include "actuators/Thruster.h"
#include "core/SimulationApp.h"
#include "core/SimulationManager.h"

namespace sf
{
/* NEW: 
    Added new way to create ocean with more parameters, including:
        wind speed, direction, and age. 
    Preserved legacy constructor by change wave type:
        "sea_state" for legacy sea state, and "params" for new parameters.
*/
 Ocean::Ocean(std::string uniqueName, Scalar waves, Fluid l,
        std::string oceanType,
        Scalar windSpeed, Scalar direction, Scalar age) : ForcefieldEntity(uniqueName)
{
    ghost->setCollisionFlags(ghost->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    oceanState = waves > Scalar(2.0) ? Scalar(2.0) : waves;

    // Set ocean type and parameters
    oceanType_ = oceanType;
    eckvWindSpeed = windSpeed;
    eckvDirection = direction;
    eckvAge = age;
     
    Scalar size(100000);
    depth = size;
    Vector3 halfExtents = Vector3(size/Scalar(2), size/Scalar(2), size/Scalar(2));
    ghost->setWorldTransform(Transform(Quaternion::getIdentity(), Vector3(0, 0, size/Scalar(2) - oceanState*Scalar(3)))); //Move ocean influence zone a bit up to account for waves
    ghost->setCollisionShape(new btBoxShape(halfExtents));
    
    currents = std::vector<VelocityField*>(0);
    currentsEnabled = false;
    
    liquid = l;
    wavesDebug.type = RenderableType::HYDRO_POINTS;
    wavesDebug.model = glm::mat4(1.f);
    wavesDebug.data = std::make_shared<std::vector<glm::vec3>>();
    waterType = Scalar(0.0);
    glOcean = nullptr;
}

Ocean::~Ocean()
{
    if(currents.size() > 0)
    {
        for(unsigned int i=0; i<currents.size(); ++i)
            delete currents[i];
        currents.clear();
    }
    
    if(glOcean != nullptr)
        delete glOcean;
}

bool Ocean::hasWaves() const
{
    // NEW: If Ocean type is set to a valid construction type, then waves are enabled.
    bool has_waves = false;
    if (oceanType_ == "params")
        has_waves = true;
    if (oceanType_ == "sea_state")
        has_waves = true;
    return has_waves;
}

bool Ocean::hasParticles() const
{
    if(glOcean != nullptr)
        return glOcean->getParticlesEnabled();
    else
        return false;
}

Scalar Ocean::getWaterType() const
{
    return waterType;
}
        
OpenGLOcean* Ocean::getOpenGLOcean()
{
    return glOcean;
}

ForcefieldType Ocean::getForcefieldType()
{
    return ForcefieldType::OCEAN;
}

Fluid Ocean::getLiquid() const
{
    return liquid;
}

VelocityField* Ocean::getVelocityField(size_t index)
{
    if(index < currents.size())
        return currents[index];
    else
        return nullptr;
}
std::vector<VelocityField*> Ocean::getVelocityFields()
{
    return currents;
}

void Ocean::setWaterType(Scalar jerlov)
{ 
    if(glOcean != nullptr)
    {
        waterType = jerlov > Scalar(1) ? Scalar(1) : (jerlov < Scalar(0) ? Scalar(0) : jerlov);
        glOcean->setWaterType((float)waterType);
    }
}

void Ocean::setParticles(bool enabled)
{
    if(glOcean != nullptr)
        glOcean->setParticles(enabled);
}

void Ocean::SetConditions(Scalar waterTemp)
{
    if(glOcean != nullptr)
        glOcean->setWaterTemperature((float)waterTemp);
}

void Ocean::AddVelocityField(VelocityField* field)
{
    currents.push_back(field);
}

bool Ocean::IsInsideFluid(const Vector3& point)
{
    return GetDepth(point) >= Scalar(0);
}

std::vector<char> Ocean::IsInsideFluidMap(const std::vector<Vector3>& points)
{
    std::vector<glm::vec3> gp(points.size());
    for(size_t i = 0; i < points.size(); ++i)
        gp[i] = glm::vec3((float)points[i].x(), (float)points[i].y(), (float)points[i].z());

    std::vector<float> d = GetDepthMap(gp);
    std::vector<char> inside(points.size());          // NOT vector<bool> (bit-packed, unsafe for parallel writes)
    for(size_t i = 0; i < d.size(); ++i)
        inside[i] = (d[i] >= 0.f) ? 1 : 0;
    return inside;
}

float Ocean::GetDepth(const glm::vec3& point)
{
    if(hasWaves()) //Geometric waves
    {
        /*
        Requires Lock since ocean update destroys and recreates glOcean
        This avoids sampling from a destroyed glOcean object giving a segfault.
        */
        SDL_LockMutex(hydroMutex_); 
        GLfloat waveHeight = glOcean->ComputeWaveHeight(point.x, point.y);
        SDL_UnlockMutex(hydroMutex_);
        glm::vec3 wavePoint(point.x, point.y, waveHeight);
        #ifdef DEBUG_WAVES
                wavesDebug.getDataAsPoints()->push_back(wavePoint);
        #endif
        
        return point.z - waveHeight;
    }
    else //Flat surface
    {
        glm::vec3 wavePoint(point.x, point.y, 0.f);
        #ifdef DEBUG_WAVES  
                wavesDebug.getDataAsPoints()->push_back(wavePoint);
        #endif
        return point.z;
    }
}

Scalar Ocean::GetDepth(const Vector3& point)
{
    return Scalar(GetDepth(glm::vec3((GLfloat)point.getX(), (GLfloat)point.getY(), (GLfloat)point.getZ())));
}

// NEW: Get depth map for a set of points
std::vector<float> Ocean::GetDepthMap(const std::vector<glm::vec3>& points)
{
    std::vector<float> depth(points.size());
    SDL_LockMutex(hydroMutex_);
    if(hasWaves())
    {
        std::vector<float> wh = glOcean->ComputeWaveHeightMap(points);   // one batched, parallel call
        for(size_t n = 0; n < points.size(); ++n)
            depth[n] = points[n].z - wh[n];
    }
    else
    {
        for(size_t n = 0; n < points.size(); ++n)
            depth[n] = points[n].z;
    }
    SDL_UnlockMutex(hydroMutex_);
    return depth;
}

Scalar Ocean::GetPressure(const Vector3& point)
{
    Scalar g = SimulationApp::getApp()->getSimulationManager()->getGravity().getZ();
    Scalar d = GetDepth(point);
    Scalar pressure = d > Scalar(0) ? d*liquid.density*g : Scalar(0);
    return pressure;
}

Vector3 Ocean::GetFluidVelocity(const Vector3& point) const
{
    if(currentsEnabled)
    {
        Vector3 fv = V0();
        for(size_t i=0; i<currents.size(); ++i)
        {
            if(currents[i]->isEnabled())
                fv += currents[i]->GetVelocityAtPoint(point);
        }
        return fv;
    }
    return V0();
}

glm::vec3 Ocean::GetFluidVelocity(const glm::vec3& point) const
{
    return glVectorFromVector(GetFluidVelocity(Vector3(point.x, point.y, point.z)));
}

void Ocean::EnableCurrents()
{
    currentsEnabled = true;
}

void Ocean::DisableCurrents()
{
    currentsEnabled = false;
}

void Ocean::UpdateCurrentsData()
{
    if(glOcean != NULL)
        glOcean->UpdateOceanCurrentsData(glOceanCurrentsUBOData);
}

void Ocean::ApplyFluidForces(btDynamicsWorld* world, btCollisionObject* co, bool recompute)
{
    Entity* ent;
    
    if (btRigidBody* rb = dynamic_cast<btRigidBody*>(co))
    {
        if (rb->isStaticOrKinematicObject())
            return;
        else
            ent = static_cast<Entity*>(rb->getUserPointer());
    }
    else if (btMultiBodyLinkCollider* mbl = dynamic_cast<btMultiBodyLinkCollider*>(co))
    {
        if (mbl->isStaticOrKinematicObject())
            return;
        else
            ent = static_cast<Entity*>(mbl->getUserPointer());
    }
    else if (btSoftBody* sb = dynamic_cast<btSoftBody*>(co))
    {
        ent = static_cast<Entity*>(sb->getUserPointer());
    }
    else
        return;
      
    HydrodynamicsSettings settings;
    
    if (ent->getType() == EntityType::SOLID)
    {
        if(recompute)
        {
            settings.dampingForces = true;
            settings.reallisticBuoyancy = true;
            ((SolidEntity*)ent)->ComputeHydrodynamicForces(settings, this);
        }
        
        ((SolidEntity*)ent)->ApplyHydrodynamicForces();
    }
    else if (ent->getType() == EntityType::CABLE)
    {
        if(recompute)
        {
            settings.dampingForces = true;
            settings.reallisticBuoyancy = true;
            ((CableEntity*)ent)->ComputeHydrodynamicForces(settings, this);
        }
        
        ((CableEntity*)ent)->ApplyHydrodynamicForces();
    }
}

void Ocean::InitGraphics(SDL_mutex* hydrodynamics)
{
    hydroMutex_ = hydrodynamics;
    if(oceanState > 0.0)
    {
        glOcean = new OpenGLRealOcean(depth, oceanState,
            oceanType_, eckvWindSpeed, eckvDirection, eckvAge, hydrodynamics);
    }
    else
        glOcean = new OpenGLFlatOcean(depth);
    setWaterType(0.2);
}

// NEW: Call OpenGLRealOcean::update() to queue the update of ocean params
// Only possible if Ocean Type is "params" to protect legacy sea state ocean type.
bool Ocean::UpdateOceanData(Scalar windSpeed, Scalar direction, Scalar age)
{
    SDL_LockMutex(hydroMutex_);
    if(glOcean == nullptr) { SDL_UnlockMutex(hydroMutex_); return false; }
    if(oceanType_ != "params")
    {
        SDL_UnlockMutex(hydroMutex_);
        cInfo("Ocean::UpdateOceanData: sea state ocean type does not support changing conditions");
        return false;
    }
    eckvWindSpeed = windSpeed;
    eckvDirection = direction;
    eckvAge = age;
    ((OpenGLRealOcean*)glOcean)->setUpdateCallback([this](){ this->ApplyPendingOceanUpdate(); });
    pendingOceanUpdate_.store(true);
    SDL_UnlockMutex(hydroMutex_);
    return true;
}

// NEW: Apply glOcean reconstruction with new params timely and without segfaults.
void Ocean::ApplyPendingOceanUpdate()
{
    cInfo("Ocean::ApplyPendingOceanUpdate: invoked");
    if(!pendingOceanUpdate_.load()) return;
    pendingOceanUpdate_.store(false);
    cInfo("Ocean::ApplyPendingOceanUpdate: rebuilding ocean (wind=%.2f, dir=%.2f, age=%.2f)",
          eckvWindSpeed, eckvDirection, eckvAge);

    float prevWaterType = (float)waterType;
    bool  prevParticles = glOcean->getParticlesEnabled();

    // 1) Snapshot the particle pointers and detach them from the old ocean
    //    so its destructor leaves them alone.
    auto particles = glOcean->getOceanParticles();  // copy of map (pointers only)
    glOcean->clearParticleMap();

    OpenGLOcean* newOcean = new OpenGLRealOcean(depth, oceanState, oceanType_,
                    eckvWindSpeed, eckvDirection, eckvAge, hydroMutex_);
    SDL_LockMutex(hydroMutex_);
    delete glOcean;
    glOcean = newOcean;
    setWaterType(prevWaterType);
    SDL_UnlockMutex(hydroMutex_);

    // 2) Re-attach the existing particle objects to the new ocean.
    for(const auto& kv : particles)
        glOcean->AssignParticles(kv.first, kv.second);

    setParticles(prevParticles);

    ((OpenGLRealOcean*)glOcean)->setUpdateCallback(nullptr);
}

std::vector<Renderable> Ocean::Render()
{
    std::vector<Actuator*> act;
    return Render(act);
}

std::vector<Renderable> Ocean::Render(const std::vector<Actuator*>& act)
{
    std::vector<Renderable> items(0);
    
    //Update currents data
    glOceanCurrentsUBOData.gravity = glm::vec3(0.f,0.f,9.81f);
    glOceanCurrentsUBOData.numCurrents = 0;

    if(currentsEnabled)
    {
        for(size_t i=0; i<currents.size(); ++i)
            if(currents[i]->isEnabled())
            {
                std::vector<Renderable> citems = currents[i]->Render(glOceanCurrentsUBOData.currents[glOceanCurrentsUBOData.numCurrents]);
                items.insert(items.end(), citems.begin(), citems.end());
                ++glOceanCurrentsUBOData.numCurrents;
            }
    }
    
    for(size_t i=0; i<act.size(); ++i)
        if(act[i]->getType() == ActuatorType::THRUSTER)
        {
            Thruster* th = (Thruster*)act[i];
            Transform thFrame = th->getActuatorFrame();
            Vector3 thPos = thFrame.getOrigin();
            Vector3 thDir = -thFrame.getBasis().getColumn(0);
            Scalar R = th->getPropellerDiameter()/Scalar(2);
            Scalar vel = (th->isPropellerRight() ? Scalar(0.1) : Scalar(-0.1)) * th->getThrust();
            glOceanCurrentsUBOData.currents[glOceanCurrentsUBOData.numCurrents].posR = glm::vec4((GLfloat)thPos.getX(), 
                                                                             (GLfloat)thPos.getY(), 
                                                                             (GLfloat)thPos.getZ(), (GLfloat)R);
            glOceanCurrentsUBOData.currents[glOceanCurrentsUBOData.numCurrents].dirV = glm::vec4((GLfloat)thDir.getX(),
                                                                             (GLfloat)thDir.getY(),
                                                                             (GLfloat)thDir.getZ(),
                                                                             (GLfloat)vel);
            glOceanCurrentsUBOData.currents[glOceanCurrentsUBOData.numCurrents].params = glm::vec3(0.f);
            glOceanCurrentsUBOData.currents[glOceanCurrentsUBOData.numCurrents].type = 10;
            ++glOceanCurrentsUBOData.numCurrents;
        }

    if(wavesDebug.getDataAsPoints()->size() > 0)
    {
        items.push_back(wavesDebug);
        wavesDebug.getDataAsPoints()->clear();
    }

    return items;
}

}
