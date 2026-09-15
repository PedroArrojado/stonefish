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
//  GustMixer.cpp
//  Stonefish
//

#include "entities/forcefields/GustMixer.h"
#include <cmath>

namespace
{
    //Cheap deterministic hash: integer cell + salt -> float in [0,1).
    //Lets every gust derive its position/strength/timing from its lattice cell,
    //so the whole field is stateless and reproducible (no stored gust list).
    inline float hashCell(int x, int y, int z, int salt)
    {
        unsigned int h = (unsigned int)(x * 374761393 + y * 668265263
                                      + z * 2147483647 + salt * 1274126177);
        h = (h ^ (h >> 13)) * 1274126177u;
        h ^= h >> 16;
        return (h & 0xFFFFFFu) / 16777216.0f;   // [0,1)
    }
}

namespace sf
{

Vector3 GustMixer::getGust(const Vector3& p, float time, const Vector3& windVel) const
{
    float speed = (float)windVel.length();
    Vector3 dir = speed > 1e-4f ? windVel / Scalar(speed) : Vector3(1, 0, 0);

    //Work in a frame that moves with the wind: blobs are static here, so in
    //world space they travel downwind at the wind speed.
    Vector3 pr = p - windVel * Scalar(time);

    const float inv = 1.0f / m_params.spacing;
    int cx = (int)std::floor((float)pr.getX() * inv);
    int cy = (int)std::floor((float)pr.getY() * inv);
    int cz = (int)std::floor((float)pr.getZ() * inv);

    const float twoSigma2 = 2.0f * m_params.radius * m_params.radius;
    float total = 0.0f;

    //Check the 2x2x2 lattice cells bracketing the point (spacing > ~2*radius
    //keeps a point within reach of at most these).
    for(int dz = 0; dz <= 1; ++dz)
      for(int dy = 0; dy <= 1; ++dy)
        for(int dx = 0; dx <= 1; ++dx)
        {
            int ix = cx + dx, iy = cy + dy, iz = cz + dz;

            //--- Temporal pulse: each cell gusts on its own phase-shifted cycle ---
            float phase = hashCell(ix, iy, iz, 4);
            float u = std::fmod(time / m_params.period + phase, 1.0f); //[0,1)
            float tloc = u * m_params.period;                          //seconds into cycle
            if(tloc >= m_params.duration)
                continue;                                              //gust off right now -> skip (also skips the exp)

            float a = tloc / m_params.duration;                        //[0,1)
            float tenv = std::sin(3.14159265f * a);
            tenv *= tenv;                                              //smooth 0 -> 1 -> 0 (raised sine)

            //--- Spatial blob: jittered center in the moving frame ---
            Vector3 center((ix + hashCell(ix, iy, iz, 1)) * m_params.spacing,
                           (iy + hashCell(ix, iy, iz, 2)) * m_params.spacing,
                           (iz + hashCell(ix, iy, iz, 3)) * m_params.spacing);
            Vector3 d = pr - center;
            float r2 = (float)d.length2();
            float sg = std::exp(-r2 / twoSigma2);

            //--- Per-gust strength variation (0.5x .. 1.0x) ---
            float amp = 0.5f + 0.5f * hashCell(ix, iy, iz, 5);

            total += amp * tenv * sg;
        }

    //Surge is along the wind direction, scaled by peak strength.
    return dir * Scalar(total * m_params.strength);
}

}
