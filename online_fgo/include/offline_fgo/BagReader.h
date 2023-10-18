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


#ifndef ONLINE_FGO_BAGREADER_H
#define ONLINE_FGO_BAGREADER_H

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "data/DataTypes.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "ublox_msgs/msg/rxm_rawx.hpp"
namespace offline_fgo {

    using namespace fgo::data_types;

    class BagReader {
        // config variables

        std::map<rclcpp::Time, PVASolution> pvaContainer_;
        std::map<rclcpp::Time, PVASolution>::iterator pvaIter_;

        std::map<rclcpp::Time, GNSSMeasurementEpoch> gnssContainer_;
        std::map<rclcpp::Time, GNSSMeasurementEpoch>::iterator gnssIter_;

        std::map<rclcpp::Time, IMUMeasurement> imuContainer_;
        std::map<rclcpp::Time, IMUMeasurement>::iterator imuIter_;

        std::string bagPath_;
        std::vector<std::string> topicFilter_;

    private: // private functions

        template<typename T>
        std::tuple<rclcpp::Time, T> _deserialize_message(const std::shared_ptr<rcutils_uint8_array_t>& data,
                                                   const rcutils_time_point_value_t & bag_timestamp)
        {
            T new_msg;
            rclcpp::Serialization<T> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*data);
            serialization.deserialize_message(
                    &extracted_serialized_msg, &new_msg);
            try {
                const auto ros_timestamp = rclcpp::Time(new_msg.header.stamp.sec,
                                                           new_msg.header.stamp.nanosec,
                                                           RCL_ROS_TIME);
                return {ros_timestamp, new_msg};
            }
            catch(std::exception &ex)
            {
                const int64_t nanoseconds_per_second = 1000000000LL;
                int32_t sec = static_cast<int32_t>(bag_timestamp / nanoseconds_per_second);
                uint32_t nanosec = static_cast<uint32_t>(bag_timestamp % nanoseconds_per_second);
                const auto ros_timestamp = rclcpp::Time(sec,nanosec,RCL_ROS_TIME);
                return {ros_timestamp, new_msg};
            }

        }

        template <typename Container, typename MemberType, typename MemberSetter>
        void populateObsFromContainer(std::vector<GNSSObs>& obs, const Container& container, MemberSetter setter)
        {
            for (const auto& value : container) {
                GNSSObs obsEntry;
                setter(obsEntry, value);
                obs.push_back(obsEntry);
            }
        }

    public:
        //constructor: initial with Path of bag file and the topics of GNSS and IMU
        explicit BagReader(std::string bagPath,
                           const  std::vector<std::string> &topicFilter) : bagPath_(std::move(bagPath)), topicFilter_(topicFilter)
                           {};

        //extract message from bag file and store it into gnss and imu container

        void parseDataFromBag();

        bool hasNextIMUMeasurement()
        {
            return imuIter_ != imuContainer_.end();
        }

        template <typename Container>
        typename Container::mapped_type getNextMeasurement(Container& container,
                                                           const std::string& name = "") {
            static auto iter = container.begin();
            if(iter != container.end())
            {
                iter++;
                return iter->second;
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("gnss_fgo"), "OfflineFGO bag_reader: at last measurement of " << name);
                return container.end()->second;
            }
        }

        template<typename Container>
        std::vector<typename Container::mapped_type> getMeasurementsBefore(Container& container,
                                                                           double timestampBefore,
                                                                           const std::string& name ="")
        {

        }

        IMUMeasurement getNextIMUMeasurement();
        GNSSMeasurementEpoch getNextGNSSMeasurement();
        PVASolution getNextPVASolution();






    };


}


#endif //ONLINE_FGO_BAGREADER_H