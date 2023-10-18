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
// Author: Lars Lünnemann (lars.luennemann@rwth-aachen.de)
//         Haoming Zhang  (h.zhang@irt.rwth-aachen.de)
//
//


#ifndef GNSSFGO_ONLINE_H
#define GNSSFGO_ONLINE_H

#pragma once


#include <irt_nav_msgs/msg/sat_label.hpp>
#include <irt_nav_msgs/msg/gnss_labeling.hpp>

//fgo
#include "GNSSFGOLocalizationBase.h"
#include "sensor/lidar/LIOSAM.h"


namespace gnss_fgo {

  class GNSSFGOTimeCentricNode : public GNSSFGOLocalizationBase{

  protected:
    // ROS Variables
    rclcpp::Subscription<irt_nav_msgs::msg::PPS>::SharedPtr subPPS_;
    rclcpp::Subscription<irt_nav_msgs::msg::PVAGeodetic>::SharedPtr subPVA_;

    // tools
    std::unique_ptr<fgo::utils::MeasurementDelayCalculator> PVTDelayCalculator_;

    //std::atomic_bool isFirstGNSSIMUSynchronized_; // For opt. on IMU, we use this bool to
  protected: //private functions
    //callback for pvtGeodetic (own thread)
    void onPVAMsgCb(irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvtMsg);

  public:
    /**
     * constructor
     */
    explicit GNSSFGOTimeCentricNode(const rclcpp::NodeOptions &opt);

    ~GNSSFGOTimeCentricNode() override
    {
      if(optThread_)
        optThread_->join();
    }

  };
} //namespace fgo




#endif //GNSSFGO_ONLINE_H
