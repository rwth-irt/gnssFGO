// /*********************************************************************
//  *
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2021
//  *  RWTH Aachen University - Institute of Automatic Control.
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of the institute nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *
//  * Author:  Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//  *********************************************************************


#ifndef ROS2_PARAMETER_H
#define ROS2_PARAMETER_H
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace utils
{
    template <class T> class RosParameter
    {
        rclcpp::Node&  node_;
        std::string    name_;
        T              value_;
    public:

        RosParameter(std::string  name, T def_value, rclcpp::Node& node):
            node_(node),
            name_(std::move(name))
        {
            if(!node_.has_parameter(name_))
                node_.declare_parameter<T>(name_, def_value);
        }

        RosParameter(std::string name, rclcpp::Node& node):
            name_(std::move(name)),
            node_(node)
        {
            if(!node_.has_parameter(name_))
                node_.declare_parameter<T>(name_);
        }

        const T& value()
        {
          node_.get_parameter(name_, value_);
          return value_;
        }
    };

}
#endif //BACHMANN_M1COM_ROS2_ROS_PARAMETER_H
