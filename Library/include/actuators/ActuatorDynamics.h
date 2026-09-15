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
//  ActuatorDynamics.h
//  Stonefish
//
//  Created by Roger Pi on 03/06/2024
//  Modified by Patryk Cieslak on 30/06/2024
//  Copyright (c) 2024 Roger Pi and Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ActuatorDynamics__
#define __Stonefish_ActuatorDynamics__

#include "StonefishCommon.h"
#include <memory>
#include "utils/SystemUtil.hpp"

namespace sf
{
    enum class RotorDynamicsType {ZERO_ORDER, FIRST_ORDER, YOEGER, BESSA, MECHANICAL_PI};
    
    //! An abstract class representing a mathematical model of rotor dynamics.
    class RotorDynamics
    {
    public:
        //! A constructor.
        RotorDynamics() : lastOutput(0), outputLimit(-1)
        {}

        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        virtual Scalar Update(Scalar dt, Scalar sp) = 0;

        //! A method returning the model type.
        virtual RotorDynamicsType getType() = 0;

        //! A method used to set the limit of output.
        /*!
          \param limit the absolute limit of the output [rad/s]
        */
        void setOutputLimit(Scalar limit)
        {
            outputLimit = limit;
        }

    protected:
        Scalar lastOutput;
        Scalar outputLimit;
    };

    // ---------- Implemententation of several models of rotor dynamics -----------

    //! A class representing a the zero order dynamics model - passthrough.
    class ZeroOrder : public RotorDynamics
    {
    public:
        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        Scalar Update(Scalar dt, Scalar sp) override
        {
            return sp;
        }

        //! A method returning the model type.
        RotorDynamicsType getType()
        {
            return RotorDynamicsType::ZERO_ORDER;
        }
    };

    //! A class representing the first order dynamics model.
    class FirstOrder : public RotorDynamics
    {
    public:
        //! A constructor
        /*!
          \param tau time constant
        */
        FirstOrder(Scalar tau) : tau(tau)
        {
        }

        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        Scalar Update(Scalar dt, Scalar sp) override
        {
            Scalar alpha = dt / tau;
            Scalar output = alpha * sp + (1 - alpha) * lastOutput;
            lastOutput = outputLimit > Scalar(0) ?  btClamped(output, -outputLimit, outputLimit) : output;
            return lastOutput;
        }

        //! A method returning the model type.
        RotorDynamicsType getType()
        {
            return RotorDynamicsType::FIRST_ORDER;
        }
    
    private:
        Scalar tau;
    };

    //! A class representing the Yoerger dynamics model.
    class Yoerger : public RotorDynamics
    {
    public:
        //! A constructor.
        /*!
          \param alpha
          \param beta
        */
        Yoerger(Scalar alpha, Scalar beta) : alpha(alpha), beta(beta)
        {
        }

        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        Scalar Update(Scalar dt, Scalar sp) override
        {
            // state += dt*(beta*_cmd - alpha*state*std::abs(state));
            Scalar output = lastOutput + dt * (beta * sp - (alpha * lastOutput * btFabs(lastOutput)));
            lastOutput = outputLimit > Scalar(0) ? btClamped(output, -outputLimit, outputLimit) : output;
            return lastOutput;
        }

        //! A method returning the model type.
        RotorDynamicsType getType()
        {
            return RotorDynamicsType::YOEGER;
        }

    private:
        Scalar alpha;
        Scalar beta;
    };

    //! A class representing the Bessa dynamics model.
    class Bessa : public RotorDynamics
    {
    public:
        //! A constructor.
        /*!
          \param Jmsp
          \param Kv1
          \param Kv2
          \param Kt
          \param Rm
        */
        Bessa(Scalar Jmsp, Scalar Kv1, Scalar Kv2, Scalar Kt, Scalar Rm) 
            : Jmsp(Jmsp), Kv1(Kv1), Kv2(Kv2), Kt(Kt), Rm(Rm)
        {
        }

        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        Scalar Update(Scalar dt, Scalar sp) override
        {
            Scalar output = lastOutput +
                dt * (sp * Kt/Rm - Kv1 * lastOutput - Kv2 * lastOutput * btFabs(lastOutput))/Jmsp;
            lastOutput = outputLimit > Scalar(0) ?  btClamped(output, -outputLimit, outputLimit) : output;
            return lastOutput;
        }

        //! A method returning the model type.
        RotorDynamicsType getType()
        {
            return RotorDynamicsType::BESSA;
        }

    private:
        Scalar Jmsp;
        Scalar Kv1;
        Scalar Kv2;
        Scalar Kt;
        Scalar Rm;
    };

    //! A classs representing a mechnical shaft model with a PI controller.
    class MechanicalPI : public RotorDynamics
    {
    public:
        //! A constructor.
        /*!
          \param J moment of inertia of the rotor [kgm2]
          \param Kp proportional gain of the PI controller [1]
          \param Ki integral gain of the PI controller [1]
          \param iLim integral limit [rad/s]
        */
        MechanicalPI(Scalar J, Scalar Kp, Scalar Ki, Scalar iLim)
            : J(J), Kp(Kp), Ki(Ki), iLim(iLim), iError(0), damping(0)
        {
        }

        //! A method that updates the model.
        /*!
          \param dt simulation time step [s]
          \param sp desired rotor angular velocity [rad/s]
        */
        Scalar Update(Scalar dt, Scalar sp) override
        {
            Scalar error = sp - lastOutput;
            Scalar tau = Kp * error + Ki * iError;
            iError = btClamped(iError + error * dt, -iLim, iLim);

            Scalar tauD = lastOutput > Scalar(0) ? damping : -damping;
            Scalar output = lastOutput + (tau - tauD)/J * dt;
            lastOutput = outputLimit > Scalar(0) ?  btClamped(output, -outputLimit, outputLimit) : output;
            return lastOutput;
        }

        //! A method used to update the damping torque.
        /*!
          \param damping absolute value of the damping torque [Nm]
        */
        void setDampingTorque(Scalar tau)
        {
            damping = btFabs(tau);
        }

        //! A method returning the model type.
        RotorDynamicsType getType()
        {
            return RotorDynamicsType::MECHANICAL_PI;
        }

    private:
        Scalar J;
        Scalar Kp;
        Scalar Ki;
        Scalar iLim;
        Scalar iError;
        Scalar damping;
    };

    // ----------------------------------------------------------------

    enum class ThrustModelType {QUADRATIC, DEADBAND, LINTERP, FD, LIFT_DRAG};

    //! An abstract class representing a mathematical model of thrust and torque generated by thrusters and propellers.
    class ThrustModel
    {
    public:
        //! A method computing the model output.
        /*!
          \param input the input to the model
          \return a pair of thrust and torque computed by the model
        */
        virtual std::pair<Scalar, Scalar> Update(Scalar input) = 0;

        //! A method returning the type of the model.
        virtual ThrustModelType getType() = 0;
    };

    // ---------- Implemententation of several models of thrust generation -----------

    //! A class representing a basic quadratic model.
    class QuadraticThrust : public ThrustModel
    {
    public:
        //! A constructor.
        /*!
          \param kt thrust coefficient
        */
        QuadraticThrust(Scalar kt) : kt(kt)
        {
        }

        //! A method computing the model output.
        /*!
          \param input the input to the model
          \return a pair of thrust and torque computed by the model
        */
        std::pair<Scalar, Scalar> Update(Scalar input) override
        {
            Scalar thrust = kt * input * btFabs(input);
            return std::make_pair(thrust, Scalar(0));
        }

        //! A method returning the type of the model.
        ThrustModelType getType() override
        {
            return ThrustModelType::QUADRATIC;
        }
    
    protected:
        Scalar kt;
    };

    //! A class representing a dead band model.
    class DeadbandThrust : public ThrustModel
    {
    public:
        //! A constructor.
        /*!
          \param ktn negative thrust coefficient
          \param ktp positive thrust coefficient
          \param dl lower limit of the deadband
          \param du upper limit of the deadband
        */
        DeadbandThrust(Scalar ktn, Scalar ktp, Scalar dl, Scalar du) : ktn(ktn), ktp(ktp), dl(dl), du(du)
        {
        }

        //! A method computing the model output.
        /*!
          \param input the input to the model
          \return a pair of thrust and torque computed by the model
        */
        std::pair<Scalar, Scalar> Update(Scalar input) override
        {
            Scalar vv = input * btFabs(input);
            Scalar thrust(0);
            if (vv < dl)
            {
                thrust = ktn * (vv - dl);
            }
            else if (vv > du)
            {
                thrust = ktp * (vv - du);
            }
            return std::make_pair(thrust, Scalar(0));
        }

        //! A method returning the type of the model.
        ThrustModelType getType() override
        {
            return ThrustModelType::DEADBAND;
        }
        
    protected:
        Scalar ktn;
        Scalar ktp;
        Scalar dl;
        Scalar du;
    };
    
    //! A class representing a model based on linear interpolation of data points.
    class InterpolatedThrust : public ThrustModel
    {
    public:
        //! A constructor.
        /*!
          \param in list of angular velocity data points
          \param out list of thrust data points
        */
        InterpolatedThrust(const std::vector<Scalar>& in, const std::vector<Scalar>& out)
            : inputValues(in), outputValues(out)
        {
            if (inputValues.empty() || outputValues.empty())
                throw std::runtime_error("Interpolated thrust model: input and output values must not be empty!");

            if (inputValues.size() != outputValues.size())
                throw std::runtime_error("Interpolated thrust model: input and output values must be same size!");
        }

        //! A method computing the model output.
        /*!
          \param input the input to the model
          \return a pair of thrust and torque computed by the model
        */
        std::pair<Scalar, Scalar> Update(Scalar input) override
        {
            Scalar thrust(0);

            // Ensure the input values are sorted
            auto it = std::lower_bound(inputValues.begin(), inputValues.end(), input);

            if (it == inputValues.begin()) // If the value is less than the smallest input value, return the first output value
            {
                thrust = outputValues.front();
            }
            else if (it == inputValues.end()) // If the value is greater than the largest input value, return the last output value
            {
                thrust = outputValues.back();
            }
            else
            {
                // Perform linear interpolation
                auto idx = std::distance(inputValues.begin(), it);
                Scalar x0 = inputValues[idx - 1];
                Scalar x1 = inputValues[idx];
                Scalar y0 = outputValues[idx - 1];
                Scalar y1 = outputValues[idx];
                thrust = y0 + (input - x0) * (y1 - y0) / (x1 - x0);
            }
            return std::make_pair(thrust, Scalar(0));
        }

        //! A method returning the type of the model.
        ThrustModelType getType() override
        {
            return ThrustModelType::LINTERP;
        }

        protected:
            std::vector<Scalar> inputValues, outputValues;
    };

    //! A class representing a realistic model based of fluid dynamics.
    class FDThrust : public ThrustModel
    {
    public:
        //! A constructor.
        /*!
          \param D diameter of the propeller [m]
          \param ktp positive thrust coefficient
          \param ktn negative thrust coefficient
          \param kq torque coefficient
          \param RH flag informing if the propeller is right-handed
        */
        FDThrust(Scalar D, Scalar ktp, Scalar ktn, Scalar kq, bool RH, Scalar rho)
            : D(D), ktp(ktp), ktn(ktn), kq(kq), RH(RH), rho(rho)
        {
            // TODO: Find a better way of defining alpha and beta
            alpha = -ktp;
            beta = -kq;
        }

        //! A method computing the model output.
        /*!
          \param input the input to the model
          \return a pair of thrust and torque computed by the model
        */
        std::pair<Scalar, Scalar> Update(Scalar input) override
        {
            bool backward = (RH && input < Scalar(0)) || (!RH && input > Scalar(0));
            
            /*kt and kq depend on the advance ratio J
                J = u/(omega*D), where:
                u - ambient velocity [m/s]
                n - propeller rotational rate [1/s]
                D - propeller diameter [m] */
            Scalar n = (backward ? Scalar(-1) : Scalar(1)) * btFabs(input)/(Scalar(2) * M_PI); // Accounts for propoller handedness
            
            // Thrust is the force generated by pushing the liquid through the thruster.
            Scalar kt0 = backward ? ktn : ktp; //In case of non-symmetrical thrusters the coefficient may be different
            //kt(J) = kt0 + alpha * J --> approximated with linear function
            Scalar thrust = rho * D*D*D * btFabs(n) * (D*kt0*n + alpha*u);
            
            // Torque is the loading of propeller due to liquid resistance (reaction force).
            Scalar kq0 = kq;
            //kQ(J) = kQ0 + beta * J --> approximated with linear function
            Scalar torque = (RH ? Scalar(-1) : Scalar(1)) * rho * D*D*D*D * btFabs(n) * (D*kq0*n + beta*u);

            return std::make_pair(thrust, torque);
        }

        //! A method used to set incoming fluid velocity.
        /*!
          \param vel velocity of the fluid coming into thruster [m/s]
        */
        void setIncomingFluidVelocity(Scalar vel)
        {
            u = vel;
        }

        //! A method returning the type of the model.
        ThrustModelType getType() override
        {
            return ThrustModelType::FD;
        }
    
    protected:
        Scalar D;
        Scalar ktp;
        Scalar ktn;
        Scalar kq;
        Scalar u;
        bool RH;
        Scalar rho;
        Scalar alpha;
        Scalar beta;
    };

    // NEW: Lift Drag model (similar to gz implementation)
     class LiftDragThrust : public ThrustModel
    {
    public:
        //! A constructor.
        /*!
        \param cLa        lift-curve slope [1/rad]
        \param cDa        drag-curve slope [1/rad]
        \param cMa        pitching-moment-curve slope [1/rad]  (reserved; forced to 0)
        \param alphaStall stall angle of attack [rad]
        \param claStall   post-stall Cl-alpha slope [1/rad]
        \param cdaStall   post-stall Cd-alpha slope [1/rad]
        \param cmaStall   post-stall Cm-alpha slope [1/rad]
        \param area       effective planform area [m^2]
        \param alpha0     zero-lift angle of attack / collective pitch offset [rad]
        \param air_rho    air density [kg/m^3]  (used when not inside liquid)
        \param cp         centre of pressure in thruster local frame [m]
        \param bladeRadius representative blade radius for tangential speed [m]
        \param rH         true = right-hand (CCW viewed from +X) propeller
        */
        LiftDragThrust(Scalar cLa, Scalar cDa, Scalar cMa, Scalar alphaStall,
                    Scalar claStall, Scalar cdaStall, Scalar cmaStall, Scalar area,
                    Scalar alpha0, Scalar kQ, Vector3 cp, bool rH)
            : cla(cLa), cda(cDa), cma(cMa), alphaStall(alphaStall),
            claStall(claStall), cdaStall(cdaStall), cmaStall(cmaStall),
            area(area), alpha0(alpha0), kQ(kQ), cp(cp), rH(rH),
            u(Scalar(0)), alpha(Scalar(0)), sweep(Scalar(0))
        {
            // forward : blade chord direction — the direction the blade moves
            //           through air (tangential, perpendicular to the thrust axis).
            //           For RH the blade tip sweeps in +Y at azimuth=0;
            //           for LH it sweeps in -Y. LH is RH with -Y forward.
            // upward  : thrust axis (+X in every Stonefish thruster frame,
            //           independent of handedness).
            // ldNormal = forward × upward  →  -Z (RH)
            //
            // With this assignment, liftDirection ≈ +X for small AoA, so lift
            // maps directly to thrust, and dragDirection ≈ -forward, so drag
            // maps to the reaction torque — exactly blade-element theory.
            if (rH)
            {
                forward = Vector3(Scalar(0),  Scalar(-1), Scalar(0));
                upward  = Vector3(Scalar(1),  Scalar(0), Scalar(0));
            }
            else
            {
                forward = Vector3(Scalar(0),  Scalar(1), Scalar(0));
                upward  = Vector3(Scalar(-1),  Scalar(0), Scalar(0));
            }
        }
    
        //! Lift/drag physics transposed from the Gazebo LiftDragPlugin, adapted
        //! to work in the thruster LOCAL frame without world-frame rotation.
        /*!
        \param vel    current rotor cp velocity vector [m/s] in local frame, including both axial inflow and tangential blade speed
        \param rho    fluid density [kg/m^3]
        \return       {force [N], torque [Nm]} both in the thruster LOCAL frame.
                        The caller must multiply by thrustTrans.getBasis() before
                        passing to ApplyCentralForce / ApplyTorque.
        */
        std::pair<Vector3, Vector3> UpdateLD(const Vector3& vel, Scalar rho)
        {
            // If the velocity is very small return zero force and torque.
            if (vel.length() <= Scalar(0.01))
                return {Vector3(0,0,0), Vector3(0,0,0)};

            // ── Lift-drag plane geometry ────────────────────────────────────────
            // ldNormal is the unit normal to the LD plane. (0,0,1) for RH, (0,0,-1) for LH.
            Vector3 ldNormal = forward.cross(upward).normalized();

            // check sweep (angle between vel and lift-drag-plane)
            Scalar sinSweep  = ldNormal.dot(vel) / vel.length();

            // get cos from trig identity
            Scalar cosSweep2 = Scalar(1) - sinSweep * sinSweep;
            sweep = btAsin(sinSweep);
            while (btFabs(sweep) > Scalar(0.5 * M_PI))
                sweep = (sweep > Scalar(0)) ? (sweep - Scalar(M_PI))
                                            : (sweep + Scalar(M_PI));

            /* angle of attack is the angle between
            vel projected into lift-drag plane and:
            forward vector projected = ldNormal Xcross ( vector Xcross ldNormal)
            so, velocity in lift-drag plane (expressed in inertial frame) is:*/
            Vector3 velInLDPlane = ldNormal.cross(vel.cross(ldNormal));

            // Aerodynamic direction vectors (local frame)
            Vector3 dragDirection   = -(velInLDPlane.normalized());
            Vector3 liftDirection   = ldNormal.cross(velInLDPlane).normalized();
            Vector3 momentDirection = ldNormal;

            alpha = alpha0 - btAtan2(u, btFabs(velInLDPlane.length()));
            // Clamp to ±90 deg
            while (btFabs(alpha) > Scalar(0.5 * M_PI))
                alpha = (alpha > Scalar(0)) ? (alpha - Scalar(M_PI))
                                            : (alpha + Scalar(M_PI));

            // ── Dynamic pressure ────────────────────────────────────────────────
            Scalar speedInLDPlane = velInLDPlane.length();
            Scalar q = Scalar(0.5) * rho * speedInLDPlane * speedInLDPlane;

            // ── Lift coefficient ────────────────────────────────────────────────
            Scalar cl;
            if (alpha > alphaStall)
                cl = std::max(Scalar(0),
                        (cla * alphaStall + claStall * (alpha - alphaStall)) * cosSweep2);
            else if (alpha < -alphaStall)
                cl = std::min(Scalar(0),
                        (-cla * alphaStall + claStall * (alpha + alphaStall)) * cosSweep2);
            else
                cl = cla * alpha * cosSweep2;

            Vector3 lift = cl * q * area * liftDirection;

            // ── Drag coefficient (always acts opposing motion) ──────────────────
            Scalar cd;
            if (alpha > alphaStall)
                cd = (cda * alphaStall + cdaStall * (alpha - alphaStall)) * cosSweep2;
            else if (alpha < -alphaStall)
                cd = (-cda * alphaStall + cdaStall * (alpha + alphaStall)) * cosSweep2;
            else
                cd = cda * alpha * cosSweep2;
            cd = btFabs(cd);    // drag is always positive (opposing)

            Vector3 drag = cd * q * area * dragDirection;

            Vector3 force = lift+drag;

            // ── Torque: X-only (yaw reaction about thrust axis) ─────────────────
            Scalar reactionMag = cd * q * area * cp.length();
            Vector3 torque(rH ? -reactionMag : reactionMag, Scalar(0), Scalar(0));
                
            if (false)
            {
                cInfo("LiftDragThrust =============================");
                cInfo("msg_counter:        %d", msg_counter++);
                cInfo("vel (local):        [%1.4f, %1.4f, %1.4f]  |v|=%1.4f",
                    vel.x(), vel.y(), vel.z(), vel.length());
                cInfo("velInLDPlane:       [%1.4f, %1.4f, %1.4f]  spd=%1.4f",
                    velInLDPlane.x(), velInLDPlane.y(), velInLDPlane.z(), speedInLDPlane);
                cInfo("ldNormal:           [%1.4f, %1.4f, %1.4f]",
                    ldNormal.x(), ldNormal.y(), ldNormal.z());
                cInfo("liftDirection:      [%1.4f, %1.4f, %1.4f]",
                    liftDirection.x(), liftDirection.y(), liftDirection.z());
                cInfo("dragDirection:      [%1.4f, %1.4f, %1.4f]",
                    dragDirection.x(), dragDirection.y(), dragDirection.z());
                cInfo("sweep:              %1.4f rad", sweep);
                cInfo("cosSweep2:          %1.4f", cosSweep2);
                cInfo("alpha:              %1.4f rad  (alpha0=%1.4f, alphaStall=%1.4f)",
                    alpha, alpha0, alphaStall);
                cInfo("dyn pressure q:     %1.4f Pa  (rho=%1.4f)", q, rho);
                cInfo("cl: %1.4f  cd: %1.4f", cl, cd);
                cInfo("lift:               [%1.4f, %1.4f, %1.4f]",
                    lift.x(), lift.y(), lift.z());
                cInfo("drag:               [%1.4f, %1.4f, %1.4f]",
                    drag.x(), drag.y(), drag.z());
                cInfo("force (local):      [%1.4f, %1.4f, %1.4f]",
                    force.x(), force.y(), force.z());
                cInfo("torque (local):     [%1.4f, %1.4f, %1.4f]",
                    torque.x(), torque.y(), torque.z());
                cInfo("cp:                 [%1.4f, %1.4f, %1.4f]",
                    cp.x(), cp.y(), cp.z());
            }

            return {force, torque};
        }
    
        //! Stub override satisfying the base-class interface.
        //! Thruster::Update calls UpdateLD() directly via a downcast.
        std::pair<Scalar, Scalar> Update(Scalar /*input*/) override
        {
            return {Scalar(0), Scalar(0)};
        }
    
        //! Set the scalar axial inflow velocity.
        /*!
        \param vel  component of (fluidVelocity − bodyVelocity) along the
                    thruster +X axis [m/s].  Positive = fluid entering
                    from the front of the thruster.
        */
        void setIncomingFluidVelocity(Scalar vel) { u = vel; }

        Vector3 cp;          //!< Centre of pressure in thruster local frame [m] public for access by Thruster for moment-arm calculation
        Scalar kQ;           //!< Torque coefficient (torque on upwaard axis = omega*|omega|*kQ) [Nm]

        //! A method returning the type of the model.
        ThrustModelType getType() override { return ThrustModelType::LIFT_DRAG; }
        
    protected:
        // Aerodynamic coefficients
        Scalar cla;          //!< Lift-curve slope [1/rad]
        Scalar cda;          //!< Drag-curve slope [1/rad]
        Scalar cma;          //!< Moment-curve slope [1/rad]
        Scalar alphaStall;   //!< Stall angle of attack [rad]
        Scalar claStall;     //!< Post-stall Cl-alpha slope [1/rad]
        Scalar cdaStall;     //!< Post-stall Cd-alpha slope [1/rad]
        Scalar cmaStall;     //!< Post-stall Cm-alpha slope [1/rad]
        int msg_counter = 0;
    
        // Geometry / configuration
        Scalar area;         //!< Planform area [m^2]
        Scalar alpha0;       //!< Zero-lift angle of attack / collective pitch offset [rad]
        bool rH;             //!< Right-hand flag
    
        // Local-frame axis vectors
        Vector3 forward;     //!< Chord / tangential direction (blade sweeps through air)
        Vector3 upward;      //!< Thrust axis (+X)
    
        // Runtime state
        Scalar u;            //!< Axial inflow velocity [m/s]
        Scalar alpha;        //!< Last computed angle of attack [rad]
        Scalar sweep;        //!< Last computed sweep angle [rad]
    };

    // ----------------------------------------------------------------
    // NEW: Control surface dynamics aggregated to thruster dynamics
    // Now all physics based actuators share the same header. 
    // refactor in the future for individual class files and an aggregator?
    // ----------------------------------------------------------------
    enum class SurfaceModelType {ANALYTIC};

    //! An abstract class representing a mathematical model of the lift and drag
    //! generated by a moveable control surface (fin/rudder/elevator/aileron).
    //! Medium-agnostic: the density of the surrounding fluid is supplied per call,
    //! so the same model type serves both air and water.
    class SurfaceModel
    {
    public:
        //! A destructor.
        virtual ~SurfaceModel() = default;

        //! A method computing the lift/drag force and moment for a given inflow.
        /*!
          \param vel relative flow velocity vector [m/s] in the surface local frame
          \param rho density of the surrounding fluid [kg/m^3]
          \return a pair {force [N], moment [Nm]} both in the surface LOCAL frame.
                  The caller must rotate them into the world frame before applying.
                  The moment is the aerodynamic pitching moment about the surface
                  reference point only; the lever-arm moment of the force about the
                  body CG is computed by the actuator, not here.
        */
        virtual std::pair<Vector3, Vector3> UpdateLD(const Vector3& vel, Scalar rho) = 0;

        //! A method returning the type of the model.
        virtual SurfaceModelType getType() = 0;
    };

    //! A class representing an analytic lift/drag control-surface model.
    //! Adapted from LiftDragThrust, with the propeller-specific rotation terms
    //! (handedness, torque coefficient kQ, spin-reaction torque) removed, since a
    //! control surface does not rotate. The lift-drag plane is defined by two
    //! configurable body-frame axes (chord and span) rather than being derived from
    //! propeller handedness, which lets one model serve a rudder, elevator, aileron
    //! or lifting fin purely by configuration.
    class AnalyticSurfaceModel : public SurfaceModel
    {
    public:
        //! A constructor.
        /*!
          \param cLa        lift-curve slope [1/rad]
          \param cDa        drag-curve slope [1/rad]
          \param alphaStall stall angle of attack [rad]
          \param claStall   post-stall Cl-alpha slope [1/rad]
          \param cdaStall   post-stall Cd-alpha slope [1/rad]
          \param area       planform area [m^2]
          \param alpha0     zero-lift angle of attack (camber offset) [rad]
          \param chordAxis  chord reference direction in the surface local frame
          \param spanAxis   span reference direction in the surface local frame
        */
        AnalyticSurfaceModel(Scalar cLa, Scalar cDa, Scalar alphaStall,
                             Scalar claStall, Scalar cdaStall, Scalar area,
                             Scalar alpha0, Vector3 chordAxis, Vector3 spanAxis)
            : cla(cLa), cda(cDa), alphaStall(alphaStall),
              claStall(claStall), cdaStall(cdaStall),
              area(area), alpha0(alpha0),
              alpha(Scalar(0)), sweep(Scalar(0))
        {
            // forward : chord direction (nominal flow-aligned reference).
            // upward  : surface normal / lift reference direction.
            // ldNormal = forward x upward = span direction; flow along the span
            //           produces sweep and contributes to drag but not lift.
            forward = chordAxis.normalized();
            Vector3 span = spanAxis.normalized();
            // Normal completes a right-handed chord/normal/span frame.
            upward = span.cross(forward).normalized();
        }

        //! Lift/drag physics adapted from LiftDragThrust::UpdateLD, without any
        //! rotor-spin terms. Returns force and (currently zero) pitching moment
        //! in the surface local frame.
        std::pair<Vector3, Vector3> UpdateLD(const Vector3& vel, Scalar rho) override
        {
            // If the velocity is very small return zero force and moment.
            if (vel.length() <= Scalar(0.01))
                return {Vector3(0,0,0), Vector3(0,0,0)};

            // ── Lift-drag plane geometry ────────────────────────────────────────
            // ldNormal is the unit normal to the LD plane (the span direction).
            Vector3 ldNormal = forward.cross(upward).normalized();

            // Sweep: angle between the flow and the lift-drag plane.
            Scalar sinSweep  = ldNormal.dot(vel) / vel.length();
            Scalar cosSweep2 = Scalar(1) - sinSweep * sinSweep;
            sweep = btAsin(sinSweep);
            while (btFabs(sweep) > Scalar(0.5 * M_PI))
                sweep = (sweep > Scalar(0)) ? (sweep - Scalar(M_PI))
                                            : (sweep + Scalar(M_PI));

            // Velocity projected into the lift-drag plane.
            Vector3 velInLDPlane = ldNormal.cross(vel.cross(ldNormal));

            // Aerodynamic direction vectors (local frame).
            Vector3 dragDirection = -(velInLDPlane.normalized());
            Vector3 liftDirection = ldNormal.cross(velInLDPlane).normalized();

            // Angle of attack: angle between the in-plane flow and the chord.
            // Unlike a rotating blade, a control surface has no tangential speed,
            // so the AoA follows directly from the flow direction in the LD plane.
            alpha = alpha0 - btAtan2(velInLDPlane.dot(upward), velInLDPlane.dot(forward));
            while (btFabs(alpha) > Scalar(0.5 * M_PI))
                alpha = (alpha > Scalar(0)) ? (alpha - Scalar(M_PI))
                                            : (alpha + Scalar(M_PI));

            // ── Dynamic pressure ────────────────────────────────────────────────
            Scalar speedInLDPlane = velInLDPlane.length();
            Scalar q = Scalar(0.5) * rho * speedInLDPlane * speedInLDPlane;

            // ── Lift coefficient (linear pre-stall, blended post-stall) ─────────
            Scalar cl;
            if (alpha > alphaStall)
                cl = std::max(Scalar(0),
                        (cla * alphaStall + claStall * (alpha - alphaStall)) * cosSweep2);
            else if (alpha < -alphaStall)
                cl = std::min(Scalar(0),
                        (-cla * alphaStall + claStall * (alpha + alphaStall)) * cosSweep2);
            else
                cl = cla * alpha * cosSweep2;

            Vector3 lift = cl * q * area * liftDirection;

            // ── Drag coefficient (always opposes motion) ────────────────────────
            Scalar cd;
            if (alpha > alphaStall)
                cd = (cda * alphaStall + cdaStall * (alpha - alphaStall)) * cosSweep2;
            else if (alpha < -alphaStall)
                cd = (-cda * alphaStall + cdaStall * (alpha + alphaStall)) * cosSweep2;
            else
                cd = cda * alpha * cosSweep2;
            cd = btFabs(cd);

            Vector3 drag = cd * q * area * dragDirection;

            Vector3 force = lift + drag;

            // Pitching moment about the surface reference point is not modelled
            // (no Cm term); the actuator applies the lever-arm moment of the force.
            Vector3 moment(Scalar(0), Scalar(0), Scalar(0));

            return {force, moment};
        }

        //! Last computed angle of attack [rad], exposed for debug/telemetry.
        Scalar getAlpha() const { return alpha; }

        //! A method returning the type of the model.
        SurfaceModelType getType() override { return SurfaceModelType::ANALYTIC; }

    protected:
        // Aerodynamic coefficients
        Scalar cla;          //!< Lift-curve slope [1/rad]
        Scalar cda;          //!< Drag-curve slope [1/rad]
        Scalar alphaStall;   //!< Stall angle of attack [rad]
        Scalar claStall;     //!< Post-stall Cl-alpha slope [1/rad]
        Scalar cdaStall;     //!< Post-stall Cd-alpha slope [1/rad]

        // Geometry / configuration
        Scalar area;         //!< Planform area [m^2]
        Scalar alpha0;       //!< Zero-lift angle of attack (camber offset) [rad]

        // Local-frame axis vectors
        Vector3 forward;     //!< Chord direction
        Vector3 upward;      //!< Surface normal (lift reference)

        // Runtime state
        Scalar alpha;        //!< Last computed angle of attack [rad]
        Scalar sweep;        //!< Last computed sweep angle [rad]
    };

} // namespace sf

#endif