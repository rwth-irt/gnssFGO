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


#ifndef ONLINE_FGO_GRAPHPARAMS_H
#define ONLINE_FGO_GRAPHPARAMS_H

#pragma once
#include <string>
#include <gtsam/geometry/Pose3.h>

#include "gnss_fgo/param/GNSSFGOParams.h"
#include "data/DataTypes.h"
#include "data/FactorTypes.h"


namespace fgo::graph
{
    using namespace fgo::data_types;
    enum OptimizationStrategy
    {
        ONIMU   =  0,
        ONGNSS  =  1
    };

    enum SmootherType
    {
        ISAM2 = 0,
        Batch = 1,
        ISAM2FixedLag = 2,
        BatchFixedLag = 3
    };

    struct GraphParamBase : gnss_fgo::GNSSFGOParams
    {
        // Graph
        double smootherLag = 0.1; // d
        SmootherType smootherType = SmootherType::ISAM2FixedLag;

        // measurement model
        //double QcGPInterpolator = 0.5;
        gtsam::Vector6 QcGPInterpolatorFull;
        //double QcGPMotionPrior = 0.5;
        gtsam::Vector6 QcGPMotionPriorFull;
        bool publishResiduals = true;
        bool onlyLastResiduals = true;
        bool AutoDiffNormalFactor = true;
        bool AutoDiffGPInterpolatedFactor = true;
        bool AutoDiffGPMotionPriorFactor = true;
        bool GPInterpolatedFactorCalcJacobian = true;

        bool useEstimatedVarianceAfterInit = true;
        bool useConstDriftFactor = false;
        data_types::NoiseModel noiseModelClockFactor = data_types::NoiseModel::GAUSSIAN;
        double robustParamClockFactor = 0.5;
        double constDriftStd = 1;
        double constBiasStd = 1;
        double angularRateStd = 1;
        double motionModelStd = 1;
        gtsam::Vector3 magnetometerStd = gtsam::Vector3(0.01,0.01,0.01);

        double IMUSensorSyncTimeThreshold = 0.005;
        double StateMeasSyncUpperBound = 0.03;
        double StateMeasSyncLowerBound = -0.03;
        bool NoOptimizationWhileNoMeasurement = false;
        bool NoOptimizationNearZeroVelocity = true;
        double VoteNearZeroVelocity = 0.1;
        int NoOptimizationAfterStates = 500;

        GraphParamBase() = default;
        explicit GraphParamBase(const gnss_fgo::GNSSFGOParamsPtr& baseParamPtr) : gnss_fgo::GNSSFGOParams(*baseParamPtr) {};
    };
    typedef std::shared_ptr<GraphParamBase> GraphParamBasePtr;

    struct GraphTimeCentricParam : GraphParamBase
    {
        bool useMMFactor = false;
        //estimation
        GraphTimeCentricParam() = default;

        explicit GraphTimeCentricParam(const GraphParamBasePtr& baseParamPtr) : GraphParamBase(*baseParamPtr){};

    };
    typedef std::shared_ptr<GraphTimeCentricParam> GraphTimeCentricParamPtr;

    struct GraphSensorCentricParam : GraphParamBase
    {
        bool useMMFactor = false;
        //estimation
        //GraphSensorCentricParam() = default;
    };
    typedef std::shared_ptr<GraphSensorCentricParam> GraphSensorCentricParamPtr;
}




#endif //ONLINE_FGO_GRAPHPARAMS_H
