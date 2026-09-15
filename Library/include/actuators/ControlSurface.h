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
//  ControlSurface.h
//  Stonefish
//


#ifndef __STONEFISH_CONTROL_SURFACE__
#define __STONEFISH_CONTROL_SURFACE__

#include "actuators/LinkActuator.h"
#include "actuators/ActuatorDynamics.h" // For SurfaceModel
#include "actuators/RotatingElement.h" // For MediumType/FluidProperties

#include "entities/forcefields/Atmosphere.h"
#include "entities/forcefields/Ocean.h"

namespace sf
{

class SolidEntity;

//! A class representing a moveable aerodynamic/hydrodynamic control surface
//! (fin/rudder/elevator/aileron). The lift/drag physics live in an injected
//! SurfaceModel, mirroring how RotatingElement delegates to a ThrustModel.
//! Up to two models may be configured, one per medium (air and water); the
//! active one is selected at runtime from the sampled fluid.
class ControlSurface : public LinkActuator
{
public:
    //! Constructor.
    /*!
      \param uniqueName a name for the control surface
      \param controlSurface a pointer to the rigid body representing the surface
      \param airModel lift/drag model used when the surface is in the atmosphere (may be nullptr)
      \param waterModel lift/drag model used when the surface is in the ocean (may be nullptr)
      \param hingeAxis deflection axis in the mount (o2a) frame
      \param cop centre of pressure in the surface local frame [m]
      \param maxAngle maximum deflection angle [rad]
      \param inverted whether the deflection setpoint is inverted
      \param maxAngularRate maximum deflection rate [rad/s] (0 = unlimited)
    */
    ControlSurface(std::string uniqueName, std::shared_ptr<SolidEntity> controlSurface,
                   std::shared_ptr<SurfaceModel> airModel,
                   std::shared_ptr<SurfaceModel> waterModel,
                   const Vector3& hingeAxis, const Vector3& cop,
                   Scalar maxAngle, bool inverted = false, Scalar maxAngularRate = 0);

    //! Destructor.
    virtual ~ControlSurface() = default;

    //! Returns the type of actuator.
    ActuatorType getType() const override;

    //! Configures active environment medium ("atmosphere", "ocean", or "hybrid").
    void setMedium(std::string medium);

    //! Queries fluid properties (density, velocity, containment) at a given point in space.
    FluidProperties getFluidAt(const Vector3& point) const;

    //! Checks whether a given point is inside the configured fluid medium.
    bool isInsideMedium(Vector3 point);

    //! Sets the surface deflection setpoint [rad].
    virtual void setSetpoint(Scalar s);

    //! Returns current surface deflection setpoint [rad].
    Scalar getSetpoint() const;

    //! Returns current actual deflection angle [rad].
    Scalar getAngle() const;

    //! Simulation step update.
    void Update(Scalar dt) override;

    //! Renders visual elements and vector debug lines.
    std::vector<Renderable> Render() override;

protected:
    //! Watchdog handler for safety shutdown.
    void WatchdogTimeout() override;

    //! Selects the lift/drag model matching the sampled medium.
    /*!
      \param fluid the sampled fluid properties at the surface
      \return the model to use, or nullptr if none is configured for that medium
    */
    std::shared_ptr<SurfaceModel> selectModel(const FluidProperties& fluid) const;

private:
    std::shared_ptr<SolidEntity> controlSurface_;

    // Environment pointers
    Atmosphere* atm_ptr{nullptr};
    Ocean* ocn_ptr{nullptr};

    // Lift/drag models, one per medium (either may be nullptr)
    std::shared_ptr<SurfaceModel> airModel;
    std::shared_ptr<SurfaceModel> waterModel;

    // Geometry / configuration
    Vector3 hingeAxis;   //!< Deflection axis in the mount frame
    Vector3 cp;          //!< Centre of pressure in the surface local frame [m]
    Scalar maxAngle;
    Scalar maxAngularRate;
    bool inv;

    // State variables
    Scalar setpoint{0};
    Scalar theta{0};

    // Calculated force vectors for rendering/debugging (surface local frame)
    Vector3 liftV{0, 0, 0};
    Vector3 dragV{0, 0, 0};
};

} // namespace sf

#endif // __STONEFISH_CONTROL_SURFACE__
