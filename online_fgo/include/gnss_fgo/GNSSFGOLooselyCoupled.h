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

//
// Created by haoming on 08.03.23.
//

#ifndef GNSSFGOLOOSELYCOUPLED_H
#define GNSSFGOLOOSELYCOUPLED_H

#pragma once

//ros
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/pps.hpp>


//fgo
#include "GNSSFGOTimeCentric.h"


namespace gnss_fgo
{

class GNSSFGOGTNode : public GNSSFGOLocalizationBase
{
protected:
    // Data and Data Buffer
    fgo::data_types::State lastlastOptimizedState_;
    rclcpp::Subscription<irt_nav_msgs::msg::PVAGeodetic>::SharedPtr subPVA_;

protected:
    //callback for pvtGeodetic (own thread)
    void onPVAMsgCb(const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr& pvtMsg);

public:
    explicit GNSSFGOGTNode(const rclcpp::NodeOptions &opt);

};

}


#endif //GNSSFGOLOOSELYCOUPLED_H
