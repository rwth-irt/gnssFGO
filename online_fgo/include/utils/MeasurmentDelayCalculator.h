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

// ToDo: the calculator should be tested again  @Haoming

#ifndef ONLINE_FGO_MEASURMENTDELAYCALCULATOR_H
#define ONLINE_FGO_MEASURMENTDELAYCALCULATOR_H

#pragma once

#include <shared_mutex>
//#include <stdatomic.h>
#include <rclcpp/rclcpp.hpp>
#include <irt_nav_msgs/msg/pps.hpp>
#include <utility>

#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"

namespace fgo::utils {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    class MeasurementDelayCalculator
    {
        std::string target_;
        std::atomic<double> tow_millisec_{};
        std::atomic_uint_fast64_t  pps_counter_{};
        double  local_delay_{};
        rclcpp::Time last_pps_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Node& node_;
        bool add_local_delay_;

        high_resolution_clock::time_point last_timer_exec_time_ = high_resolution_clock::now();

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_mutex mutex_;
        std::unique_ptr<CalculateMeasurementDelay> calculator_;
        CalculateMeasurementDelay::ExtU_CalculateMeasurementDela_T calculator_input_{};
        CalculateMeasurementDelay::ExtY_CalculateMeasurementDela_T calculator_output_{};

    public:

        explicit MeasurementDelayCalculator(rclcpp::Node& node,
                                            const rclcpp::CallbackGroup::SharedPtr& group,
                                            bool add_local_delay = true,
                                            std::string target = "") : node_(node), add_local_delay_(add_local_delay), target_(std::move(target))
        {

          calculator_ = std::make_unique<CalculateMeasurementDelay>();
          calculator_->initialize();

          timer_ = node_.create_wall_timer(std::chrono::duration<double>( 0.001),
                                          [this]()->void {
            mutex_.lock();

            calculator_->setExternalInputs(&calculator_input_);
            calculator_->step();

            calculator_output_ = calculator_->getExternalOutputs();
            //std::cout << "pps: " << calculator_input_.TimePuls << " TOW: " << calculator_input_.TOW << " Delay: " << calculator_->getExternalOutputs().MeasurementDelay << std::endl;
            mutex_.unlock();
            //std::cout << "hz: " << (1./std::chrono::duration_cast<std::chrono::duration<double>>( now - last).count()) << std::endl;
            /* Getting number of milliseconds as a double. */
            //auto now = high_resolution_clock::now();
            //duration<double, std::milli> ms_double = now - last_timer_exec_time_;
            //std::cout << ms_double.count() << "ms\n";
            //last_timer_exec_time_ = now;
            },
          group);
        }

        void setPPS(const irt_nav_msgs::msg::PPS::ConstSharedPtr msg)
        {
          rclcpp::Time thisTime(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
          pps_counter_ = msg->pps_counter;
          mutex_.lock();
          calculator_input_.TimePuls = msg->pps_counter;
          mutex_.unlock();
          local_delay_ = (thisTime - last_pps_time_).seconds() - 1.;
          //RCLCPP_INFO_STREAM(appPtr_.get_logger(), "Calculate Delay: local delay in s " << local_delay_);
          last_pps_time_ = thisTime;
        }

        void setTOW(double tow)
        {
          //RCLCPP_INFO_STREAM(appPtr_.get_logger(), "TOW: " << std::fixed << tow);
          tow_millisec_ = tow;
          mutex_.lock();
          calculator_input_.TOW = tow;
          mutex_.unlock();
        }

        [[nodiscard]] double getDelay()
        {
          double delay;
          mutex_.lock_shared();
          delay = calculator_output_.MeasurementDelayCalc; // + (add_local_delay_ ? local_delay_ : 0.);
          mutex_.unlock_shared();
         // RCLCPP_WARN_STREAM(appPtr_.get_logger(), target_ << " intern delay: " << calculator_output_.MeasurementDelayCalc);
          return delay;
        }
    };
}
#endif //ONLINE_FGO_MEASURMENTDELAYCALCULATOR_H
