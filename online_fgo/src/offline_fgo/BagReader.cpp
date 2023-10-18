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
// Created by haoming on 7/2/23.
//

#include "offline_fgo/BagReader.h"

namespace offline_fgo
{
    void BagReader::parseDataFromBag() {

        // now we create a reader to get all data
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options{};
        storage_options.uri = bagPath_;
        storage_options.storage_id = "sqlite3";
        rosbag2_cpp::ConverterOptions converter_options{};
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        reader.open(storage_options, converter_options);

        if(!topicFilter_.empty())
        {
            auto filter_ = rosbag2_storage::StorageFilter();
            filter_.topics = topicFilter_;
            reader.set_filter(filter_);
        }

        while (reader.has_next()) {
            auto bag_message = reader.read_next();
            auto message_topic_name = bag_message->topic_name;
            //IMU,topic:"/imu/data" , type:sensor_msgs::msg::Imu
            if (message_topic_name == topicFilter_[1]) {

                const auto [timestamp, imu_msg] = this->_deserialize_message<sensor_msgs::msg::Imu>(bag_message->serialized_data,
                                                                                                    bag_message->time_stamp);
                IMUMeasurement imuMeasurement;
                imuMeasurement.timestamp = timestamp;
                imuMeasurement.accLin.x() = imu_msg.linear_acceleration.x;
                imuMeasurement.accLin.y() = imu_msg.linear_acceleration.y;
                imuMeasurement.accLin.z() = imu_msg.linear_acceleration.z;
                imuMeasurement.accLinCov = gtsam::Vector9(imu_msg.angular_velocity_covariance.data());
                imuMeasurement.AHRSOri.w() = imu_msg.orientation.w;
                imuMeasurement.AHRSOri.x() = imu_msg.orientation.x;
                imuMeasurement.AHRSOri.y() = imu_msg.orientation.y;
                imuMeasurement.AHRSOri.z() = imu_msg.orientation.z;
                imuMeasurement.gyro.x() = imu_msg.angular_velocity.x;
                imuMeasurement.gyro.y() = imu_msg.angular_velocity.y;
                imuMeasurement.gyro.z() = imu_msg.angular_velocity.z;
                imuMeasurement.accLinCov = gtsam::Vector9(imu_msg.angular_velocity_covariance.data());
                imuMeasurement.AHRSOriCov = gtsam::Vector9(imu_msg.orientation_covariance.data());
                imuContainer_.insert(std::make_pair(imuMeasurement.timestamp, imuMeasurement));

            }
            //GNSS , topic:"/ublox/rxmrawx" , type:ublox_msgs::msg::RxmRAWX
            else if (message_topic_name == topicFilter_[0]) {
                const auto [timestamp, gnss_msg] = this->_deserialize_message<irt_msgs::msg::GNSSObsPreProcessed>
                        (bag_message->serialized_data,bag_message->time_stamp);
                GNSSMeasurementEpoch gnssMeasurement;
                gnssMeasurement.timestamp = timestamp;
                gnssMeasurement.tow = gnss_msg.gnss_obs_ant_main.time_receive;
                const auto& pseudorange = gnss_msg.gnss_obs_ant_main.pseudorange;
                const auto& satellitePos = gnss_msg.gnss_obs_ant_main.satelite_pos;
                const auto& satelliteVel = gnss_msg.gnss_obs_ant_main.satelite_vec;
                for (std::size_t i = 0; i < 6; ++i) {
                    GNSSObs obsEntry;
                    obsEntry.pr = pseudorange[i];
                    obsEntry.satPos.x() = satellitePos[i].x;
                    obsEntry.satPos.y() = satellitePos[i].y;
                    obsEntry.satPos.z() = satellitePos[i].z;
                    obsEntry.satVel.x() = satelliteVel[i].x;
                    obsEntry.satVel.y() = satelliteVel[i].y;
                    obsEntry.satVel.z() = satelliteVel[i].z;
                    gnssMeasurement.obs.push_back(obsEntry);
                }
                gnssContainer_.insert(std::make_pair(gnssMeasurement.timestamp, gnssMeasurement));

            }
        }
    }

    IMUMeasurement BagReader::getNextIMUMeasurement() {
        imuIter_++;
        return getNextMeasurement(imuContainer_, "imu");
    }

    GNSSMeasurementEpoch BagReader::getNextGNSSMeasurement() {
        return getNextMeasurement(gnssContainer_, "gnss");
    }

    PVASolution BagReader::getNextPVASolution() {
        return getNextMeasurement(pvaContainer_, "pva");
    }

}




