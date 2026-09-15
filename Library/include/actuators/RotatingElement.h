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
//  RotatingElement.h
//  Stonefish
//


#ifndef __STONEFISH_ROTATING_ELEMENT__
#define __STONEFISH_ROTATING_ELEMENT__

#include "actuators/LinkActuator.h"
#include "actuators/ActuatorDynamics.h"

#include "entities/forcefields/Atmosphere.h"
#include "entities/forcefields/Ocean.h"

namespace sf
{

class SolidEntity;

//! Helper structure representing fluid environment state at a specific query point.
enum class MediumType { NONE, ATMOSPHERE, OCEAN };
struct FluidProperties { 
    bool isInside=false; 
    Scalar density=0; 
    Vector3 velocity; 
    MediumType medium=MediumType::NONE; 
};

//! A class representing a rotating propulsor (combining marine thrusters and aerial propellers).
class RotatingElement : public LinkActuator
{
public:
    //! Constructor.
    /*!
    \param uniqueName        name of the actuator
    \param propeller         pointer to the visual/physical propeller entity
    \param rotorDynamics    pointer to the rotor motor/dynamics model
    \param airThrust        thrust conversion model used in the atmosphere (may be nullptr)
    \param waterThrust      thrust conversion model used in the ocean (may be nullptr)
    \param diameter         propeller/rotor diameter [m]
    \param rightHand        true if right-hand rotation, false for left-hand
    \param maxSetpoint      maximum velocity setpoint limit [rad/s]
    \param invertedSetpoint invert setpoint direction flag
    \param normalizedSetpoint setpoint expected in range [-1, 1] flag
    */
    RotatingElement(std::string uniqueName, std::shared_ptr<SolidEntity> propeller,
                    std::shared_ptr<RotorDynamics> rotorDynamics,
                    std::shared_ptr<ThrustModel> airThrust,
                    std::shared_ptr<ThrustModel> waterThrust,
                    Scalar diameter, bool rightHand, Scalar maxSetpoint,
                    bool invertedSetpoint = false, bool normalizedSetpoint = false);

    //! Destructor.
    virtual ~RotatingElement() = default;

    //! Returns the type of actuator.
    ActuatorType getType() const override;

    //! Configures active environment medium ("atmosphere", "ocean", or "hybrid").
    void setMedium(std::string medium);

    //! Queries fluid properties (density, velocity, containment) at a given point in space.
    FluidProperties getFluidAt(const Vector3& point) const;

    //! Checks whether a given point is inside the configured fluid medium.
    bool isInsideMedium(Vector3 point);

    //! Sets the rotor speed setpoint [rad/s or normalized].
    virtual void setSetpoint(Scalar s);

    //! Sets the rotor maximum speed limit [rad/s].
    void setSetpointLimit(Scalar limit);

    //! Returns the current setpoint limit [rad/s].
    Scalar getSetpointLimit();

    //! Returns the current target setpoint.
    Scalar getSetpoint() const;

    //! Returns the current angular position [rad].
    Scalar getAngle() const;

    //! Returns the current rotational velocity [rad/s].
    Scalar getOmega() const;

    //! Returns the current thrust magnitude [N].
    Scalar getThrust() const;

    //! Returns the current reaction torque magnitude [Nm].
    Scalar getTorque() const;

    //! Returns true if propeller handedness is right-hand (CW/CCW orientation).
    bool isPropellerRight() const;

    //! Returns propeller diameter [m].
    Scalar getPropellerDiameter() const;

    //! Simulation step update.
    void Update(Scalar dt) override;

    //! Renders visual elements and vector debug lines.
    std::vector<Renderable> Render() override;

protected:
    //! Watchdog handler for safety shutdown.
    void WatchdogTimeout() override;

    //! Selects the thrust model matching the sampled medium.
    /*!
      \param fluid the sampled fluid properties at the propeller
      \return the model to use, or nullptr if none is configured for that medium
    */
    std::shared_ptr<ThrustModel> selectModel(const FluidProperties& fluid) const;

private:
    // Physical & Model references
    std::shared_ptr<SolidEntity> propeller_;
    std::shared_ptr<RotorDynamics> rotorModel;
    std::shared_ptr<ThrustModel> airThrustModel;   //!< Thrust model used in atmosphere
    std::shared_ptr<ThrustModel> waterThrustModel; //!< Thrust model used in ocean

    // Environment pointers
    Atmosphere* atm_ptr{nullptr};
    Ocean* ocn_ptr{nullptr};

    // Propeller parameters
    bool RH;             //!< Handedness flag
    Scalar D;            //!< Propeller diameter [m]

    // State variables
    Scalar theta;        //!< Accumulated rotation angle [rad]
    Scalar omega;        //!< Rotational speed [rad/s]
    Scalar thrust;       //!< Calculated thrust magnitude [N]
    Scalar torque;       //!< Calculated reaction torque magnitude [Nm]

    // Setpoint variables
    Scalar setpoint;
    Scalar setpointLimit;
    bool inv;
    bool normalized;

    // Local force vectors for rendering/debugging
    Vector3 forceLocal{0, 0, 0};
    Vector3 torqueLocal{0, 0, 0};
    Vector3 forceGlobal{0, 0, 0};
    Vector3 torqueGlobal{0, 0, 0};
};

} // namespace sf

#endif // __STONEFISH_ROTATING_ELEMENT__