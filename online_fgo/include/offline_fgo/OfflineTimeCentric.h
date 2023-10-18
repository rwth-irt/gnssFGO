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

#ifndef ONLINE_FGO_OFFLINETIMECENTRIC_H
#define ONLINE_FGO_OFFLINETIMECENTRIC_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "BagReader.h"
#include "online_fgo/params/OnlineFGOParams.h"
#include "graphs/GraphTimeCentric.h"
#include "graphs/integrators/IntegratorBase.h"
#include "graphs/GraphBase.h"
#include "utils/GNSSUtils.h"
#include "utils/ROSParameter.h"
#include "GNSSFGOLocalizationBase.h"


//third party
#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"
#include "InitGyroBias_ert_rtw/InitGyroBias.h"
#include "InitStatePVT_ert_rtw/InitStatePVT.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
using gtsam::symbol_shorthand::N;  // integer ambiguities
using gtsam::symbol_shorthand::M;  // integer ddambiguities
using gtsam::symbol_shorthand::A;  // acceleration
using gtsam::symbol_shorthand::O;

namespace offline_fgo {
    using namespace fgo::integrator;
    using namespace fgo::graph;
    class OfflineFGOTimeCentric : public online_fgo::OnlineFGOBase{

        std::shared_ptr<BagReader> reader_;
        fgo::graph::GraphTimeCentric::Ptr graph_{};
        //Preintegrated IMU Measurements
        std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> currentIMUPreintegrator_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;

        //data container
        std::map<rclcpp::Time,fgo::data_types::State> fgoPredStateContainer_;
        std::map<rclcpp::Time,fgo::data_types::State> fgoOptStateContainer_;
        std::map<size_t, double> statetimeContainer_;
        fgo::data_types::State lastOptimizedState_;
        fgo::data_types::State currentPredState_;
        std::vector<std::array<double, 3>> initGyroBias_;

        //Ros variable
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStatePredPub_{};
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStateOptPub_{};
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Subscription<irt_nav_msgs::msg::GNSSObsPreProcessed>::SharedPtr subGnss_{};
        rclcpp::Publisher<irt_nav_msgs::msg::ElapsedTimeFGO>::SharedPtr TimerPub_{};
        std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callbackGroupMap_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStatePredNavFixPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStateOptNavFixPub_;
        // initialize parameter

        std::shared_ptr<std::thread> optThread_;
        std::shared_ptr<std::thread> initFGOThread_;
        std::atomic_bool lastInitFinished_{};
        std::atomic_bool isStateInited_{};

        //vector data
        std::vector<std::pair<rclcpp::Time, double>> referenceSensorTimestampContainer_;

        //threading management
        bool triggeredInit_ = false;

        //tools
        std::unique_ptr<InitGyroBias> gyroBiasInitializer_;
        std::unique_ptr<InitStatePVT> statePVTInitializer_;
        std::unique_ptr<fgo::utils::MeasurementDelayCalculator> GNSSDelayCalculator_;

    public:
        //function
        explicit OfflineFGOTimeCentric(const rclcpp::NodeOptions& opt);
        ~OfflineFGOTimeCentric();


    private:
        void doOfflineFGOProcess();


    };


}





#endif //ONLINE_FGO_OFFLINETIMECENTRIC_H
