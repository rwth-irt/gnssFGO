// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef GNSS_FGO_ONLINEFGOPARAMS_H
#define GNSS_FGO_ONLINEFGOPARAMS_H

#pragma once

#include <string>
#include "data/DataTypes.h"

namespace gnss_fgo{
    struct GNSSFGOParams
    {
        bool verbose = false;
        bool useIMUAsTimeReference = false;
        int optFrequency = 10; // 10 Hz
        int stateFrequency = 10;
        int bufferSize = 100;

        bool calibGravity = true;
        bool calcErrorOnOpt = false;
        bool delayFromPPS = false;
        double pvtMeasTimeOffset = 0.;

        double accelerometerSigma = 0.0008;
        double integrationSigma = 1e-4;
        double gyroscopeSigma = 0.00052;
        double biasAccSigma = 0.0004;
        double biasOmegaSigma = 0.00087;
        double biasAccOmegaInt = 0.00001;
        uint IMUMeasurementFrequency = 100;  // 10 Hz

        bool useGPPriorFactor = false;
        bool useGPInterpolatedFactor = false;
        fgo::data_types::GPModelType gpType = fgo::data_types::GPModelType::WNOA;

        gtsam::Point3 transIMUToCorrevit = gtsam::Point3(0, 0, 0);
        gtsam::Point3 transIMUToReference = gtsam::Point3(0, 0, 0);

        gtsam::Rot3 rotIMUToReference = gtsam::Rot3();
        gtsam::Point3 transIMUToAnt1 = gtsam::Point3(0, 0, 0);  // lever arm between imu and phase center of main antenna
        gtsam::Point3 transIMUToAnt2 = gtsam::Point3(0, 0, 0);
        gtsam::Point3 transIMUToLiDAR = gtsam::Point3(0.339560, 0, -0.0787);
        gtsam::Rot3 rotIMUtoLiDAR = gtsam::Rot3::Roll(-M_PI);

        bool UsePPSTimeSync = true;

        // Initializer
        bool initGyroBiasAsZero = true;
        bool cleanIMUonInit = true;
        bool useHeaderTimestamp = true;
        gtsam::Rot3 imuRot = gtsam::Rot3::Identity();

        // publisher
        //bool pubNavFixAntMain = true;
        //bool pubNavFixAntAux = false;

        virtual ~GNSSFGOParams()
        = default;
    };
    typedef std::shared_ptr<GNSSFGOParams> GNSSFGOParamsPtr;
}

#endif //GNSS_FGO_ONLINEFGOPARAMS_H
