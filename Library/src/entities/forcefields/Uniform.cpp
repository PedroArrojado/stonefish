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
//  Uniform.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 2/05/19.
//  Copyright(c) 2019-2025 Patryk Cieslak. All rights reserved.
//

#include "entities/forcefields/Uniform.h"

namespace sf
{

// NEW: Constructor for the Uniform class now includes mixers
Uniform::Uniform(const Vector3& velocity)
{
    setVelocity(velocity);
    TurbulenceMixer::Params t;
    t.frequency = 0.0;
    t.persistence = 0.0;
    t.scale = 0.0;
    t.strength = 0.0;

    //GENTLE
    // t.strength    = 0.4f;   // ~20% of 2 m/s base
    // t.scale       = 0.15f;  // ~6–7 m eddies, well above grid floor
    // t.frequency   = 0.15f;  // slow temporal churn — field nearly steady
    // t.persistence = 0.5f;   // (still inert)

    //MODERATE
    // t.strength    = 0.7f;   // ~35% of base
    // t.scale       = 0.22f;  // ~4–5 m eddies
    // t.frequency   = 0.3f;   // gentle evolution
    // t.persistence = 0.5f;

    //AGGRESSIVE
    // t.strength    = 1.0f;   // ~50% of base — visible swirl, still net downstream drift
    // t.scale       = 0.3f;   // ~3 m eddies (approaching grid floor)
    // t.frequency   = 0.5f;   // faster but not strobing
    // t.persistence = 0.5f;

    m_turbulence.setParams(t);

    GustMixer::Params g;
    g.strength  = 0.0f;
    g.radius    = 0.0f;
    g.spacing   = 0.0f;
    g.period    = 0.0f;
    g.duration  = 0.0f;

    //Occasional strong gusts
    // g.strength  = 1.8f;   // strong surge, just under the 2 m/s base
    // g.radius    = 10.0f;   // large, easy-to-see blobs
    // g.spacing   = 14.f;   // sparse — usually 0–1 gust in view at a time
    // g.period    = 8.f;    // long cycle
    // g.duration  = 2.0f;   // ~25% duty — brief gusts, long quiet

    // Steady breeze with frequent light gusts
    // g.strength  = 0.8f;   // gentle surges
    // g.radius    = 2.5f;
    // g.spacing   = 7.f;    // denser — several blobs in view
    // g.period    = 5.f;
    // g.duration  = 3.0f;   // ~60% duty — usually something gusting somewhere

    // Punchy squalls
    // g.strength  = 1.5f;
    // g.radius    = 2.0f;   // tighter blobs (still above the ~1.25 m grid floor)
    // g.spacing   = 10.f;
    // g.period    = 4.f;
    // g.duration  = 1.0f;   // ~25% duty, short cycle — quick stabs

    m_gust.setParams(g);
}

VelocityFieldType Uniform::getType() const
{
    return VelocityFieldType::UNIFORM;
}

void Uniform::setVelocity(const Vector3& x)
{
    v = x;
}

Vector3 Uniform::getVelocity() const
{
    return v;
}

// NEW: Get the velocity at a point, including turbulence and gusts if enabled
Vector3 Uniform::GetVelocityAtPoint(const Vector3& p) const
{
    Vector3 v_at_p = v;
    if(turbulence_enabled) 
        v_at_p += m_turbulence.getTurbulence(p, current_time);
    if(gust_enabled) 
        v_at_p += m_gust.getGust(p, current_time, v);

    return v_at_p;
}

std::vector<Renderable> Uniform::Render(VelocityFieldUBO& ubo)
{
    std::vector<Renderable> items(0);
    Scalar vel = v.length();
    Vector3 dir = vel > Scalar(0) ? (v/vel) : Vector3(0,0,0);
    ubo.posR = glm::vec4(0.f);
    ubo.dirV = glm::vec4((GLfloat)dir.getX(), (GLfloat)dir.getY(), (GLfloat)dir.getZ(), (GLfloat)vel);
    ubo.params= glm::vec3(0.f);
    ubo.type = 0;
    return items;
}

}

