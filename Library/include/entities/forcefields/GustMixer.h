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
//  GustMixer.h
//  Stonefish
//
//  Stochastic, spatially-localized wind gusts: soft blobs placed on a
//  wind-advected jittered lattice, each pulsing on its own random cycle.
//  Additive surge along the wind direction. Stateless (hash-based), so it is
//  reproducible and safe to evaluate in parallel.
//

#ifndef __Stonefish_GustMixer__
#define __Stonefish_GustMixer__

#include "StonefishCommon.h"

namespace sf
{
    //! A class generating stochastic, moving wind gusts as an additive velocity bump.
    class GustMixer
    {
    public:
        //! Tunable parameters for the gust field.
        struct Params
        {
            float strength;   //!< peak surge speed of a gust [m/s]
            float radius;     //!< spatial size of a gust blob (Gaussian sigma) [m]
            float spacing;    //!< mean distance between gust centers [m]
            float period;     //!< length of each blob's on/off cycle [s]
            float duration;   //!< how long a gust is active within its cycle [s]

            Params() : strength(1.5f), radius(2.5f), spacing(10.f),
                       period(6.f), duration(2.5f) {}
        };

        GustMixer() {}
        explicit GustMixer(const Params& p) : m_params(p) {}

        //! Additive gust velocity at a point.
        /*!
         \param p the world-space sample point [m]
         \param time the current field time [s]
         \param windVel the base wind velocity (sets travel speed and surge direction) [m/s]
         \return additive velocity contribution [m/s]
         */
        Vector3 getGust(const Vector3& p, float time, const Vector3& windVel) const;

        void setParams(const Params& p) { m_params = p; }
        const Params& getParams() const { return m_params; }

    private:
        Params m_params;
    };
}

#endif
