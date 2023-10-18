//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//


#ifndef CONSTANTS_H
#define CONSTANTS_H

#pragma once

#include <vector>
#include <string>

namespace fgo::constants {
        const std::vector<std::string> OPTSTRATEGIES = {"onTimer", "onGNSS"};
        const uint64_t sec2nanosec = 1e9;
        const int speedOfLight = 299792458;   // [m/s]
        const double gravity = 9.80665;   // [m/s*s]
        const double L1 = 1575.42e+6;   // GPS L1 freq [HZ]
        const double lambda_L1 = 0.19029367279;
        const double L2 = 1227.6e+6;   // GPS L2 freq [Hz]
        const double L5 = 1176.45e+6;   // GPS L5 freq [Hz]
        const double semiMajor = 6378137.0;  // Earth's Semi-Major Axis [m]
        const double semiMinor = 6356752.31424518;    // Earth's Semi-Minor Axis [m]
        const double earthRot = 7.2921151467e-5;     // Earth's rotation rate [rad/sec]
        const int stdPressure = 1013;   // Std Atmosphere Pressure [mbar]
        const double deg2rad = M_PI / 180.;
        const double rad2deg = 180. / M_PI;
        //const double stdTemp = 288.15;    // Std Atmosphere Temp. [Kelvin]
        //const double mu = 3.986004418e+14; //WGS84 Earth gravitational constant (m^3 s^-2)
        //const double J_2 = 1.082627e-3; //%WGS84 Earth's second gravitational constant
        //const double omega_ie = 7.292115e-5;

        enum PVTType
        {
            NO_SOLUTION  = 0,
            SINGLE       = 1,
            PSRDIFF      = 2,
            WAAS         = 3,
            NARROW_FLOAT = 5,
            NARROW_INT   = 4,
            PPP          = 6
        };

}

#endif