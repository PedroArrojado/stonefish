
#include "entities/forcefields/TurbulenceMixer.h"
#include "SimplexNoise.h"

namespace sf
{
Vector3 TurbulenceMixer::getTurbulence(const Vector3& p, float time) const
{
    return computeCurl(p, time);
}

Vector3 TurbulenceMixer::computeCurl(const Vector3& p, float time) const
{
    const float eps   = 0.1f;                        // finite-difference step [m]
    const float scale = m_params.scale;
    const float tOff  = time * m_params.frequency;   // frequency now drives temporal evolution

    // Vector potential Ψ(p): three independent 3D noise fields.
    // Large fixed offsets decorrelate the components; domain scaled by `scale`,
    // animated by translating through noise space at `tOff`.
    auto potentialAt = [](const sf::Vector3& wp, float scale, float tOff)
    {
        float x = wp.getX() * scale + tOff;
        float y = wp.getY() * scale + tOff;
        float z = wp.getZ() * scale + tOff;

        float a = SimplexNoise::noise(x,           y,           z);
        float b = SimplexNoise::noise(x + 31.416f, y + 27.183f, z + 19.265f);
        float c = SimplexNoise::noise(x + 53.702f, y + 61.314f, z + 11.937f);
        return sf::Vector3(a, b, c);
    };

    // Sample the potential just either side of p along each axis
    Vector3 px1 = potentialAt(p + Vector3(eps,0,0), scale, tOff);
    Vector3 px0 = potentialAt(p - Vector3(eps,0,0), scale, tOff);
    Vector3 py1 = potentialAt(p + Vector3(0,eps,0), scale, tOff);
    Vector3 py0 = potentialAt(p - Vector3(0,eps,0), scale, tOff);
    Vector3 pz1 = potentialAt(p + Vector3(0,0,eps), scale, tOff);
    Vector3 pz0 = potentialAt(p - Vector3(0,0,eps), scale, tOff);

    const float inv = 1.0f / (2.0f * eps);

    // curl = ( ∂Ψz/∂y - ∂Ψy/∂z , ∂Ψx/∂z - ∂Ψz/∂x , ∂Ψy/∂x - ∂Ψx/∂y )
    float curlX = (py1.getZ() - py0.getZ())*inv - (pz1.getY() - pz0.getY())*inv;
    float curlY = (pz1.getX() - pz0.getX())*inv - (px1.getZ() - px0.getZ())*inv;
    float curlZ = (px1.getY() - px0.getY())*inv - (py1.getX() - py0.getX())*inv;

    return Vector3(curlX, curlY, curlZ) * m_params.strength;
}
}
