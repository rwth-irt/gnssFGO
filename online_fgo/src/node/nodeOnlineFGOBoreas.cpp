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


#include <rclcpp/rclcpp.hpp>
#include "gnss_fgo/GNSSFGOBoreas.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nodeOptions = rclcpp::NodeOptions();

    auto node = std::make_shared<gnss_fgo::GNSSFGOBoreasNode>(nodeOptions);
    static const size_t THREAD_NUM = 7; // Receive and blocking command service.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), THREAD_NUM);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
  return 0;
}