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
//  Anemometer.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 09/06/2014.
//  Copyright (c) 2014-2025 Patryk Cieslak. All rights reserved.
//

#include "sensors/scalar/Anemometer.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "sensors/Sample.h"

namespace sf
{

Anemometer::Anemometer(std::string uniqueName, Scalar frequency, int historyLength) : 
                LinkSensor(uniqueName, frequency, historyLength)
{
    channels.push_back(SensorChannel("Anemometer", QuantityType::WIND));
}

void Anemometer::InternalUpdate(Scalar dt)
{
    Vector3 apparentWind(0, 0, 0);

    Atmosphere* atm = SimulationApp::getApp()->getSimulationManager()->getAtmosphere();
    if (atm != nullptr)
    {
        // 1. Get world position & true wind velocity
        Vector3 sensorPos = getSensorFrame().getOrigin();
        Vector3 trueWind = atm->GetFluidVelocity(sensorPos);

        // 2. Subtract robot/sensor linear velocity to get apparent wind in World Frame
        Vector3 sensorVel = attach->getLinearVelocity(); // Robot/sensor velocity in world space
        Vector3 apparentWindWorld = trueWind - sensorVel;

        // 3. Rotate the wind vector into the local sensor coordinate frame
        apparentWind = quatRotate(getSensorFrame().getRotation().inverse(),apparentWindWorld);
    }

    // Record sample (X, Y, Z components in m/s)
    Sample s{std::vector<Scalar>({apparentWind.getX(), apparentWind.getY(), apparentWind.getZ()})};
    AddSampleToHistory(s);
}

SensorType Anemometer::getType() const
{
    return SensorType::OTHER;
}

ScalarSensorType Anemometer::getScalarSensorType() const
{
    return ScalarSensorType::ANEMOMETER;
}

}
