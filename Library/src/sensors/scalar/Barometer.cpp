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
//  Barometer.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 02/11/2017.
//  Copyright (c) 2017-2021 Patryk Cieslak. All rights reserved.
//

#include "sensors/scalar/Barometer.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "sensors/Sample.h"

namespace sf
{

Barometer::Barometer(std::string uniqueName, Scalar frequency, int historyLength) : LinkSensor(uniqueName, frequency, historyLength)
{
    channels.push_back(SensorChannel("Barometer", QuantityType::PRESSURE));
}

void Barometer::InternalUpdate(Scalar dt)
{
    Scalar data(0.); //Gauge Barometer //data(101325.); //Pa (1 atm)
    
    Atmosphere* atm = SimulationApp::getApp()->getSimulationManager()->getAtmosphere();
    if(atm != NULL)
        data += atm->GetPressure(getSensorFrame().getOrigin());
    
    //Record sample
    Sample s{std::vector<Scalar>({data})};
    AddSampleToHistory(s);
}

void Barometer::setRange(Scalar max)
{
    channels[0].rangeMin = Scalar(0);
    channels[0].rangeMax = btClamped(max, Scalar(0), Scalar(BT_LARGE_FLOAT));
}
    
void Barometer::setNoise(Scalar BarometerStdDev)
{
    channels[0].setStdDev(btClamped(BarometerStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
}

ScalarSensorType Barometer::getScalarSensorType() const
{
    return ScalarSensorType::BAROMETER;
}

}
