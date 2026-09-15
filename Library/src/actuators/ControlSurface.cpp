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
//  ControlSurface.cpp
//  Stonefish
//


#include "actuators/ControlSurface.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLContent.h"
#include "entities/SolidEntity.h"

namespace sf
{

ControlSurface::ControlSurface(std::string uniqueName, std::shared_ptr<SolidEntity> controlSurface,
                               std::shared_ptr<SurfaceModel> airModel,
                               std::shared_ptr<SurfaceModel> waterModel,
                               const Vector3& hingeAxis, const Vector3& cop,
                               Scalar maxAngle, bool inverted, Scalar maxAngularRate)
    : LinkActuator(uniqueName), controlSurface_(controlSurface),
      airModel(airModel), waterModel(waterModel),
      hingeAxis(hingeAxis.normalized()), cp(cop),
      maxAngle(btFabs(maxAngle)), maxAngularRate(btFabs(maxAngularRate)),
      inv(inverted), setpoint(Scalar(0)), theta(Scalar(0))
{
    if (controlSurface_)
        controlSurface_->BuildGraphicalObject();
}

ActuatorType ControlSurface::getType() const
{
    return ActuatorType::CONTROL_SURFACE;
}

void ControlSurface::setMedium(std::string medium)
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

FluidProperties ControlSurface::getFluidAt(const Vector3& point) const
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

bool ControlSurface::isInsideMedium(Vector3 point)
{
    return getFluidAt(point).isInside;
}

std::shared_ptr<SurfaceModel> ControlSurface::selectModel(const FluidProperties& fluid) const
{
    // Prefer the model matching the sampled medium. If only one model is
    // configured, use it regardless of medium (the common single-medium case).
    if (fluid.medium == MediumType::ATMOSPHERE && airModel)
        return airModel;
    if (fluid.medium == MediumType::OCEAN && waterModel)
        return waterModel;
    if (airModel && !waterModel)
        return airModel;
    if (waterModel && !airModel)
        return waterModel;
    return nullptr;
}

void ControlSurface::setSetpoint(Scalar s)
{
    if (inv) s *= Scalar(-1);
    setpoint = btClamped(s, -maxAngle, maxAngle);
    ResetWatchdog();
}

Scalar ControlSurface::getSetpoint() const
{
    return inv ? -setpoint : setpoint;
}

Scalar ControlSurface::getAngle() const
{
    return theta;
}

void ControlSurface::Update(Scalar dt)
{
    Actuator::Update(dt);

    // Update deflection angle subject to the angular-rate limit
    if (maxAngularRate > Scalar(0) && btFabs(setpoint - theta) / dt > maxAngularRate)
    {
        Scalar dTheta = (setpoint - theta > Scalar(0)) ? maxAngularRate * dt : -maxAngularRate * dt;
        theta += dTheta;
    }
    else
    {
        theta = setpoint;
    }

    if (attach == nullptr)
        return;

    // Deflection about the configurable hinge axis, applied in the mount frame.
    Quaternion surfaceRot(hingeAxis, theta);

    // Transforms
    Transform solidTrans = attach->getCGTransform();
    Transform surfaceTrans = attach->getOTransform() * o2a * Transform(surfaceRot);

    // Sample the fluid at the centre of pressure, expressed in world coordinates.
    Vector3 cpWorld = surfaceTrans * cp;
    FluidProperties fluid = getFluidAt(cpWorld);

    std::shared_ptr<SurfaceModel> model = selectModel(fluid);

    if (!fluid.isInside || model == nullptr)
    {
        liftV = Vector3(0, 0, 0);
        dragV = Vector3(0, 0, 0);
        return;
    }

    // Relative flow velocity at the centre of pressure, in the surface local frame.
    Vector3 relPos = cpWorld - solidTrans.getOrigin();
    Vector3 absVel = attach->getLinearVelocityInLocalPoint(relPos);
    Vector3 velLocal = surfaceTrans.getBasis().transpose() * (fluid.velocity - absVel);

    // Delegate the lift/drag physics to the model (medium-agnostic; density passed in).
    Vector3 forceLocal, momentLocal;
    std::tie(forceLocal, momentLocal) = model->UpdateLD(velLocal, fluid.density);

    // Rotate force and moment into the world frame.
    Matrix3 R = surfaceTrans.getBasis();
    Vector3 forceWorld = R * forceLocal;
    Vector3 momentWorld = R * momentLocal;

    // Lever-arm moment of the force about the body CG.
    Vector3 leverTorque = relPos.cross(forceWorld);

    // Apply force at CG plus the resulting and aerodynamic moments.
    attach->ApplyCentralForce(forceWorld);
    attach->ApplyTorque(leverTorque);
    attach->ApplyTorque(momentWorld);

    // Cache lift/drag split for rendering (drag opposes flow, remainder is lift).
    Vector3 flowLocal = velLocal;
    if (!flowLocal.isZero())
    {
        Vector3 dragDir = flowLocal.normalized();
        Scalar dragMag = forceLocal.dot(dragDir);
        dragV = dragMag * dragDir;
        liftV = forceLocal - dragV;
    }
    else
    {
        dragV = Vector3(0, 0, 0);
        liftV = forceLocal;
    }
}

std::vector<Renderable> ControlSurface::Render()
{
    Transform CSTrans = Transform::getIdentity();
    if(attach != NULL)
        CSTrans = attach->getOTransform() * o2a;
    else
        LinkActuator::Render();
    
    //Rotate rudder
    CSTrans *= Transform(Quaternion(theta, 0, 0)) * controlSurface_->getO2GTransform();
    
    //Add renderable
    std::vector<Renderable> items(0);
    Renderable item;
    item.type = RenderableType::SOLID;
    item.materialName = controlSurface_->getMaterial().name;
    item.objectId = controlSurface_->getGraphicalObject();
    item.lookId = dm == DisplayMode::GRAPHICAL ? controlSurface_->getLook() : -1;
	item.model = glMatrixFromTransform(CSTrans);
    items.push_back(item);
    
    item.type = RenderableType::ACTUATOR_LINES;
    item.data = std::make_shared<std::vector<glm::vec3>>();
    auto points = item.getDataAsPoints();
    points->push_back(glm::vec3(0,0,0));
    Vector3 VG = .1*(controlSurface_->getO2GTransform().inverse().getBasis()*(liftV + dragV));
    points->push_back(glm::vec3(VG.getX(),VG.getY(),VG.getZ()));
    items.push_back(item);
    
    return items;
}

void ControlSurface::WatchdogTimeout()
{
    setSetpoint(Scalar(0));
}

} // namespace sf
