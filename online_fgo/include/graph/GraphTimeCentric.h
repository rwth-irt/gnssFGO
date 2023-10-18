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

#ifndef ONLINE_FGO_GRAPHTIMECENTRIC_H
#define ONLINE_FGO_GRAPHTIMECENTRIC_H

#pragma once

#include <any>
#include "graph/GraphBase.h"
#include "data/DataTypes.h"
#include "integrator/GNSSTCIntegrator.h"
#include "integrator/GNSSLCIntegrator.h"
#include "integrator/LIOIntegrator.h"

namespace fgo::graph
{
    class GraphTimeCentric : public GraphBase//,  std::enable_shared_from_this<GraphTimeCentric>
    {
        GraphTimeCentricParamPtr paramPtr_;
        rclcpp::Publisher<irt_nav_msgs::msg::SensorProcessingReport>::SharedPtr pubIMUFactorReport_;

    public:
        typedef std::shared_ptr<GraphTimeCentric> Ptr;

        explicit GraphTimeCentric(gnss_fgo::GNSSFGOLocalizationBase& node);

        ~GraphTimeCentric() override
        {
            if(pubResidualsThread_)
            pubResidualsThread_->join();
        };

        StatusGraphConstruction constructFactorGraphOnIMU(
            std::vector<fgo::data_types::IMUMeasurement>& dataIMU
        ) override;

        StatusGraphConstruction constructFactorGraphOnTime(
            const std::vector<double>& stateTimestamps,
            std::vector<fgo::data_types::IMUMeasurement> &dataIMU
        ) override;


        double optimize(fgo::data_types::State& new_state) override;
    };
}



#endif //ONLINE_FGO_GRAPHTIMECENTRIC_H
