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
//  RotatingElement.cpp
//  Stonefish
//


#include "actuators/RotatingElement.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLContent.h"
#include "entities/SolidEntity.h"

namespace sf
{

RotatingElement::RotatingElement(std::string uniqueName, std::shared_ptr<SolidEntity> propeller,
                                 std::shared_ptr<RotorDynamics> rotorDynamics,
                                 std::shared_ptr<ThrustModel> airThrust,
                                 std::shared_ptr<ThrustModel> waterThrust,
                                 Scalar diameter, bool rightHand, Scalar maxSetpoint,
                                 bool invertedSetpoint, bool normalizedSetpoint)
    : LinkActuator(uniqueName), RH(rightHand), D(diameter),
      theta(Scalar(0)), omega(Scalar(0)), thrust(Scalar(0)), torque(Scalar(0)),
      setpoint(Scalar(0)), setpointLimit(maxSetpoint), inv(invertedSetpoint), normalized(normalizedSetpoint),
      rotorModel(rotorDynamics), airThrustModel(airThrust), waterThrustModel(waterThrust),
      ocn_ptr(nullptr), atm_ptr(nullptr)
{
    setSetpointLimit(maxSetpoint);
    propeller_ = propeller;
    propeller_->BuildGraphicalObject();
}

std::shared_ptr<ThrustModel> RotatingElement::selectModel(const FluidProperties& fluid) const
{
    // The sampled medium tag is authoritative: return the model configured for
    // that medium, or nullptr if the propeller is outside any fluid or no model
    // is configured for the medium it is currently in.
    if (fluid.medium == MediumType::ATMOSPHERE)
        return airThrustModel;
    if (fluid.medium == MediumType::OCEAN)
        return waterThrustModel;
    return nullptr;
}

ActuatorType RotatingElement::getType() const
{
    return ActuatorType::ROTATING_ELEMENT;
}

void RotatingElement::setMedium(std::string medium)
{
    auto* mgr = SimulationApp::getApp()->getSimulationManager();
    if (medium == "atmosphere")
    {
        atm_ptr = mgr->getAtmosphere();
        ocn_ptr = nullptr;
    }
    else if (medium == "ocean")
    {
        ocn_ptr = mgr->getOcean();
        atm_ptr = nullptr;
    }
    else if (medium == "hybrid")
    {
        atm_ptr = mgr->getAtmosphere();
        ocn_ptr = mgr->getOcean();
    }
    else
    {
        cInfo("Invalid Actuation medium type");
    }
}

FluidProperties RotatingElement::getFluidAt(const Vector3& point) const
{
    FluidProperties fp;

    // Check if its inside Ocean, otherwise its in Atmosphere
    if (ocn_ptr && ocn_ptr->IsInsideFluid(point))
    {
        fp.isInside = true;
        fp.density = ocn_ptr->getLiquid().density;
        fp.velocity = ocn_ptr->GetFluidVelocity(point);
        fp.medium = MediumType::OCEAN;
        return fp;
    }
    else if(atm_ptr)
    {
        fp.isInside = true;
        fp.density = atm_ptr->getGas().density;
        fp.velocity = atm_ptr->GetFluidVelocity(point);
        fp.medium = MediumType::ATMOSPHERE;
        return fp;
    }

    return fp;
}

bool RotatingElement::isInsideMedium(Vector3 point)
{
    return getFluidAt(point).isInside;
}

void RotatingElement::setSetpoint(Scalar s)
{
    if (normalized)
        setpoint = btClamped(s, Scalar(-1), Scalar(1)) * setpointLimit;
    else
        setpoint = btClamped(s, -setpointLimit, setpointLimit);
    if (inv) setpoint *= Scalar(-1);
    ResetWatchdog();
}

void RotatingElement::setSetpointLimit(Scalar limit)
{
    setpointLimit = btFabs(limit);
    rotorModel->setOutputLimit(setpointLimit * 2); // Protect against uncontrolled behavior
}

Scalar RotatingElement::getSetpointLimit()
{
    return setpointLimit;
}

Scalar RotatingElement::getSetpoint() const
{
    return inv ? -setpoint : setpoint;
}

Scalar RotatingElement::getAngle() const
{
    return theta;
}

Scalar RotatingElement::getOmega() const
{
    return omega;
}

Scalar RotatingElement::getThrust() const
{
    return thrust;
}

Scalar RotatingElement::getTorque() const
{
    return torque;
}

bool RotatingElement::isPropellerRight() const
{
    return RH;
}

Scalar RotatingElement::getPropellerDiameter() const
{
    return D;
}

void RotatingElement::Update(Scalar dt)
{
    Actuator::Update(dt);

    if (attach == nullptr)
        return; // No attachment, no action

    // Update rotation & angular velocity
    if (rotorModel->getType() == RotorDynamicsType::MECHANICAL_PI)
        std::static_pointer_cast<MechanicalPI>(rotorModel)->setDampingTorque(btFabs(torque));
    
    omega = rotorModel->Update(dt, setpoint);
    theta += omega * dt; // Visual update

    Transform solidTrans = attach->getCGTransform();
    Transform thrustTrans = attach->getOTransform() * o2a;

    // Abstracted Fluid Fetch
    FluidProperties fluid = getFluidAt(thrustTrans.getOrigin());

    // Select the thrust model for the medium the propeller is currently in.
    std::shared_ptr<ThrustModel> thrustModel = selectModel(fluid);

    if (thrustModel == nullptr)
    {
        thrust = Scalar(0);
        torque = Scalar(0);
        forceLocal = Vector3(0, 0, 0);
        torqueLocal = Vector3(0, 0, 0);
        return;
    }

    // ── LIFT-DRAG MODEL (Aero/Hydro Blade Element) ─────────────────────────
    if (thrustModel->getType() == ThrustModelType::LIFT_DRAG)
    {
        auto ldModel = std::static_pointer_cast<LiftDragThrust>(thrustModel);
        Scalar u(0);

        if (fluid.isInside)
        {
            Vector3 relPos = thrustTrans.getOrigin() - solidTrans.getOrigin();
            Vector3 velocity = attach->getLinearVelocityInLocalPoint(relPos);
            u = -thrustTrans.getBasis().getColumn(0).dot(fluid.velocity - velocity);
            ldModel->setIncomingFluidVelocity(u);
        }
        else
        {
            ldModel->setIncomingFluidVelocity(Scalar(0));
        }

        Scalar r = ldModel->cp.length();
        Scalar tangential = RH ? -(omega * r) : (omega * r);
        Vector3 velLocal(u, tangential, Scalar(0));

        // Compute local forces and reaction torque
        Vector3 torque_from_spin(-(ldModel->kQ * omega * btFabs(omega)), Scalar(0), Scalar(0));
        Matrix3 R = thrustTrans.getBasis();
        
        std::tie(forceLocal, torqueLocal) = ldModel->UpdateLD(velLocal, fluid.density);
        
        forceGlobal = R * forceLocal;
        torqueGlobal = R * (torqueLocal + torque_from_spin);

        Vector3 armWorld = thrustTrans.getOrigin() - solidTrans.getOrigin();
        Vector3 leverTorqueGlobal = armWorld.cross(forceGlobal);

        // Apply physical forces
        attach->ApplyCentralForce(forceGlobal);
        attach->ApplyTorque(leverTorqueGlobal);   // Off-CG thrust moment
        attach->ApplyTorque(torqueGlobal);        // Reaction torque

        thrust = forceLocal.getX();
        torque = (torqueLocal + torque_from_spin).getZ();
    }
    // ── EMPIRICAL / STANDARD THRUSTER MODELS ─────────────────────────────────
    else
    {
        if (fluid.isInside)
        {
            if (thrustModel->getType() == ThrustModelType::FD)
            {
                Vector3 relPos = thrustTrans.getOrigin() - solidTrans.getOrigin();
                Vector3 velocity = attach->getLinearVelocityInLocalPoint(relPos);
                Scalar u = -thrustTrans.getBasis().getColumn(0).dot(fluid.velocity - velocity);
                std::static_pointer_cast<FDThrust>(thrustModel)->setIncomingFluidVelocity(u);
            }

            std::pair<Scalar, Scalar> out = thrustModel->Update(omega);
            thrust = out.first;
            torque = out.second;

            if (!RH && thrustModel->getType() != ThrustModelType::FD)
                thrust = -thrust;

            Vector3 thrustV(thrust, 0, 0);
            Vector3 torqueV(torque, 0, 0);

            attach->ApplyCentralForce(thrustTrans.getBasis() * thrustV);
            attach->ApplyTorque((thrustTrans.getOrigin() - solidTrans.getOrigin()).cross(thrustTrans.getBasis() * thrustV));
            attach->ApplyTorque(thrustTrans.getBasis() * torqueV);
        }
        else
        {
            thrust = Scalar(0);
            torque = Scalar(0);
        }
    }
}

std::vector<Renderable> RotatingElement::Render()
{
    Transform RETrans = Transform::getIdentity();
    if (attach != nullptr)
        RETrans = attach->getOTransform() * o2a;
    else
        LinkActuator::Render();

    // Rotate propeller
    RETrans *= Transform(Quaternion(0, 0, theta), Vector3(0, 0, 0));

    // Add renderable
    std::vector<Renderable> items(0);
    Renderable item;
    item.type = RenderableType::SOLID;
    item.materialName = propeller_->getMaterial().name;
    item.objectId = propeller_->getGraphicalObject();
    item.lookId = dm == DisplayMode::GRAPHICAL ? propeller_->getLook() : -1;
    item.model = glMatrixFromTransform(RETrans);
    items.push_back(item);

    item.type = RenderableType::ACTUATOR_LINES;
    item.data = std::make_shared<std::vector<glm::vec3>>();
    auto points = item.getDataAsPoints();
    points->push_back(glm::vec3(0, 0, 0));
    points->push_back(glm::vec3(0.1f * thrust, 0, 0));
    items.push_back(item);

    return items;
}

void RotatingElement::WatchdogTimeout()
{
    setSetpoint(Scalar(0));
}

} // namespace sf