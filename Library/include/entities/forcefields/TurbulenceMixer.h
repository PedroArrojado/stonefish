#ifndef __Stonefish_TurbulenceMixer__
#define __Stonefish_TurbulenceMixer__

#include "StonefishCommon.h"

namespace sf
{
    //! TurbulenceMixer utility class.
    /*!
     Calculates divergence-free curl noise to perturb velocity fields.
     */
    class TurbulenceMixer
    {
    public:
        struct Params {
            float strength;  // Multiplier for the noise
            float scale;     // Spatial frequency
            float frequency; // Temporal frequency (time evolution)
            float persistence; // Octave blending

            Params() : strength(1.5f), scale(2.5f), frequency(10.f),
                       persistence(6.f) {}
        };

        TurbulenceMixer() {}
        explicit TurbulenceMixer(const Params& p) : m_params(p) {}
        
        //! Computes the turbulence vector at point p.
        Vector3 getTurbulence(const Vector3& p, float time) const;

        void setParams(const Params& p) { m_params = p; }
        const Params& getParams() const { return m_params; }
        
    private:
        Params m_params;
        
        // Helper to compute the curl of the noise field
        Vector3 computeCurl(const Vector3& p, float time) const;
    };
}
#endif