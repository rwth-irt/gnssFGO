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


#ifndef ONLINE_FGO_INTEGRATORPARAMS_H
#define ONLINE_FGO_INTEGRATORPARAMS_H

#pragma once

#include <string>
#include <gtsam/geometry/Point3.h>
#include "graph/param/GraphParams.h"

namespace fgo::integrator::param
{
    struct IntegratorBaseParams : fgo::graph::GraphParamBase
    {
        bool notIntegrating = false;
        double StateSensorSyncTimeThreshold = 0.03;  // if the time difference of GNSS and State i is less than this seconds, they are synchronized
        bool useHeaderTimestamp = true;
        bool useForInitialization = false;
        fgo::factor::MeasurementFrame attitudeFrame = fgo::factor::MeasurementFrame::NED;
        fgo::factor::AttitudeType attitudeType = fgo::factor::AttitudeType::RPY;
        data_types::NoiseModel noiseModelAttitude = data_types::NoiseModel::GAUSSIAN;
        double robustParamAttitude = 0.5;
        fgo::factor::MeasurementFrame velocityFrame = fgo::factor::MeasurementFrame::BODY;
        fgo::factor::VelocityType velocityType = fgo::factor::VelocityType::VEL3D;
        data_types::NoiseModel noiseModelVelocity = data_types::NoiseModel::GAUSSIAN;
        double robustParamVelocity = 0.5;

        data_types::NoiseModel noiseModelOdomPose = data_types::NoiseModel::GAUSSIAN;
        double robustParamOdomPose = 0.5;
        gtsam::Vector6 odomPoseVar = (gtsam::Vector6() << 0.5, 0.5, 0.5, 2, 2, 2).finished();

        bool integrateVelocity = true;   // not read in integratorBase
        bool integrateAttitude = true;   // not read in integratorBase
        bool integrateRollPitch = false; // not read in integratorBase

        bool hasRoll = false;            // not read in integratorBase
        bool hasPitch = false;          // not read in integratorBase
        bool hasHeading = false;
        // uncertainty variables
        double velVarScale = 1.;        // not read in integratorBase
        double headingVarScale = 1.;    // not read in integratorBase
        double varScaleHeadingRTKFloat = 1.;
        double varScaleHeadingSingle = 1.;
        double fixedVelVar = 1.;        // not read in integratorBase
        gtsam::Vector3 velVar = gtsam::Vector3(0.1,0.1,0.1);

        double zeroVelocityThreshold = 0.05;

        IntegratorBaseParams() = default;

        explicit IntegratorBaseParams(const fgo::graph::GraphParamBasePtr& baseParamPtr) : fgo::graph::GraphParamBase(*baseParamPtr) {};
    };
    typedef std::shared_ptr<IntegratorBaseParams> IntegratorBaseParamsPtr;

    struct IntegratorGNSSTCParams : IntegratorBaseParams
    {
        uint GNSSMeasurementFrequency = 10;  // 10 Hz
        const double lambdaL1 = fgo::constants::speedOfLight / fgo::constants::L1;

        bool addCBDPriorFactorOnNoGNSS = true;
        bool usePseudoRangeDoppler = false;
        bool usePseudoRange = false;
        bool useDopplerRange = false;
        bool useDDCarrierPhase = false;
        bool useDDPseudoRange = false;
        bool useTDCarrierPhase = false;
        bool useDualAntenna = false;
        bool useDualAntennaDD = false;
        bool useRTCMDD = false;
        bool pseudorangeUseRawStd = false;
        int pseudorangeFactorTil = 10;
        double carrierStdScale = 1.;
        int thresholdSatNumToCreateTDCP = 2;
        double pseudorangeVarScaleAntMain = 2400;
        double dopplerrangeVarScaleAntMain = 8;
        double pseudorangeVarScaleAntAux= 3600;
        double dopplerrangeVarScaleAntAux = 16;
        double carrierphaseStd = 0.01;
        double initCovforIntAmb = 100000.0;
        std::string weightingModel = "STD";
        double ddCPStart = 10; //seconds to wait until DDCP factor start

        data_types::NoiseModel noiseModelPRDR = data_types::NoiseModel::GAUSSIAN;
        data_types::NoiseModel noiseModelDDCP = data_types::NoiseModel::GAUSSIAN;
        data_types::NoiseModel noiseModelTDCP = data_types::NoiseModel::GAUSSIAN;

        double robustParameterPRDR = 0.5;
        double robustParameterDDCP = 0.5;
        double robustParameterTDCP = 0.5;

        IntegratorGNSSTCParams() = default;
        explicit IntegratorGNSSTCParams(const IntegratorBaseParamsPtr &baseParamPtr) : IntegratorBaseParams(*baseParamPtr){}
    };
    typedef std::shared_ptr<IntegratorGNSSTCParams> IntegratorGNSSTCParamsPtr;

    struct IntegratorGNSSLCParams : IntegratorBaseParams
    {
        uint GNSSMeasurementFrequency = 10;  // 10 Hz
        double posVarScale = 1.;

        double varScaleRTKFloat = 1.;
        double varScaleSingle = 1.;
        double varScaleNoSolution = 1000.;

        //std::string PVTSource = "oem7";
        data_types::NoiseModel noiseModelPosition = data_types::NoiseModel::GAUSSIAN;

        bool onlyRTKFixed = true;
        double robustParamPosition = 0.5;
        double heading_offset_deg = 0.;


        IntegratorGNSSLCParams() = default;
        explicit IntegratorGNSSLCParams(const IntegratorBaseParamsPtr &baseParamPtr)  : IntegratorBaseParams(*baseParamPtr){}

    };
    typedef std::shared_ptr<IntegratorGNSSLCParams> IntegratorGNSSLCParamsPtr;

    struct IntegratorOdomParams : IntegratorBaseParams
    {
        bool integrateBetweenPose = true;
        bool integrateGlobalPose = false;

        IntegratorOdomParams() = default;
        explicit IntegratorOdomParams(const IntegratorBaseParamsPtr &baseParamPtr) : IntegratorBaseParams(*baseParamPtr){}
    };
    typedef std::shared_ptr<IntegratorOdomParams> IntegratorOdomParamsPtr;

    struct IntegratorCorrevitParams : IntegratorBaseParams {
        //IntegratorCorrevitParams() = default;

        double nearZeroVelocityThreshold = 0.;
        double factorizeDelay = 5.;
        bool enablePreIntegration = false;

        explicit IntegratorCorrevitParams(const IntegratorBaseParamsPtr &baseParamPtr) : IntegratorBaseParams(*baseParamPtr){}
    };
    typedef std::shared_ptr<IntegratorCorrevitParams> IntegratorCorrevitParamsPtr;

    struct IntegratorIRTPVAParams : IntegratorBaseParams {
        IntegratorIRTPVAParams() = default;
        explicit IntegratorIRTPVAParams(const IntegratorBaseParamsPtr &baseParamPtr) : IntegratorBaseParams(*baseParamPtr){}
    };
    typedef std::shared_ptr<IntegratorIRTPVAParams> IntegratorIRTPVAParamsPtr;
}

#endif //ONLINE_FGO_INTEGRATORPARAMS_H
