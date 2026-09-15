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
//  Altimeter.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 02/11/2017.
//  Copyright (c) 2017-2021 Patryk Cieslak. All rights reserved.
//

#include "sensors/scalar/Altimeter.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "sensors/Sample.h"

namespace sf
{

Altimeter::Altimeter(std::string uniqueName, Scalar frequency, int historyLength) : LinkSensor(uniqueName, frequency, historyLength)
{
    channels.push_back(SensorChannel("Altimeter", QuantityType::LENGTH));
}

void Altimeter::InternalUpdate(Scalar dt)
{
    // 1. Get raw position height (Assuming +Z up, or -Z down)
    Scalar data = -getSensorFrame().getOrigin().getZ();

    // 2. Clamp to configured range limits [0, rangeMax]
    if (channels[0].rangeMax > Scalar(0))
    {
        if (data > channels[0].rangeMax || data < channels[0].rangeMin)
        {
            // Out-of-range value (e.g. max range reading or 0)
            data = channels[0].rangeMax; 
        }
    }

    // 4. Record sample
    Sample s{std::vector<Scalar>({data})};
    AddSampleToHistory(s);
}

void Altimeter::setRange(Scalar max)
{
    channels[0].rangeMin = Scalar(0);
    channels[0].rangeMax = btClamped(max, Scalar(0), Scalar(BT_LARGE_FLOAT));
}

void Altimeter::setNoise(Scalar AltimeterStdDev)
{
    channels[0].setStdDev(btClamped(AltimeterStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
}

ScalarSensorType Altimeter::getScalarSensorType() const
{
    return ScalarSensorType::ALTIMETER;
}

}
