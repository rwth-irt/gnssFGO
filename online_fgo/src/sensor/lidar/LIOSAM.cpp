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

#include "sensor/lidar/LIOSAM.h"

using namespace std::chrono_literals;

namespace sensors::LiDAR::LIOSAM
{
    void LIOSAMOdometry::laserCloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msg) {
      const auto timestampCloudInfo = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
      if(isFirstScan_)
      {
        odomInfoMsg_->header = msg->header;
        this->allocateMemory();
        firstScanTimestamp_ = timestampCloudInfo;
        RCLCPP_WARN_STREAM(node_.get_logger(), "!!!!!!!!!!!!!!!!! FIRST SCAN AT " << std::fixed <<timestampCloudInfo.seconds());
        lastQueryStateOutput_ = this->queryLiDARPoseFromOptPose(timestampCloudInfo);

        if(!lastQueryStateOutput_.queriedSuccess)
        {
          RCLCPP_ERROR_STREAM(node_.get_logger(), "!!!!!!!!!!!!!!!!! CANT QUERY POSE FOR THE FIRST SCAN! " << timestampCloudInfo.seconds());
          return;
        }

        isFirstScan_ = false;
        currentCloudInfo_ = *msg;
        odomInfoMsg_->counter = odomInfoCounter_++;
        pcl::fromROSMsg(msg->cloud_corner, *laserCloudCornerLast_);
        pcl::fromROSMsg(msg->cloud_surface, *laserCloudSurfLast_);

        cloudKeyPoseIndexTimestampMap_.insert(std::make_pair(0, firstScanTimestamp_.seconds()));

        this->downsampleCurrentScan();
        this->updateKeyCloudPoseImpl_(0, gtsam::Pose3(), firstScanTimestamp_.seconds());

        timestampLastScan_ = timestampCloudInfo.seconds();
        timestampLastKeyPose_ = timestampCloudInfo.seconds();
        //lastIMUPoseTrue = imuStateBuffer_.get_buffer(firstScanTimestamp_).state.pose();
        return;
      }

      if(!skipScan_ && (timestampCloudInfo.seconds() - timestampLastScan_ >= params_.mappingProcessInterval)) {
        RCLCPP_WARN_STREAM(node_.get_logger(), "!!!!!!!!!!!!!!!!! NEW SCAN AT " << std::fixed <<timestampCloudInfo.seconds());
        lidarInputBuffer_.update_buffer(*msg, timestampCloudInfo);
        timestampLastScan_ = timestampCloudInfo.seconds();
      }
    }

    void LIOSAMOdometry::processLidarInput() {

      while(rclcpp::ok())
      {
          static size_t scanCounter = 1;  //
          static bool skippedOptimization = false;
          static nav_msgs::msg::Odometry odomIncremental;
          static Eigen::Affine3d increOdomAffine; // incremental odometry in affine
          static gtsam::Pose3 last_pose;
          //static fgo::data_types::QueryStateOutput lastQueryStateOutput;
          static gtsam::Pose3 lastIMUPoseTrue;
          rclcpp::sleep_for(10ms);

          if(isFirstScan_)
              // if on first scan, caching it
              continue;

          const auto lidarInputSize = lidarInputBuffer_.size();

          if(skipScan_)
          {
              keyPoseCounter_ = cloudKeyPoses3D_->size();
              lastOptFinished_ = true;
              continue;
          }

          if(!lidarInputSize)
          {
              //RCLCPP_INFO(node_.get_logger(), "[LIOSAM]: no lidar data cached!");
              continue;
          }

          if(!lastOptFinished_)
          {
              //RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: last optimization not finished! DOING NOTHING...");
              continue;
          }


          if(keyPoseCounter_ != cloudKeyPoses3D_->size())
          {
              RCLCPP_WARN_STREAM(node_.get_logger(), "[LIOSAM]: the cloud key pose was NOT updated for last LiDAR odom " << keyPoseCounter_ <<
                                                                                                                     "Cloud Size: " << cloudKeyPoses3D_->size()<< " DOING NOTHING...");
              continue;
          }

          irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
          thisProcessingReport.sensor_name = "LiDAROdom";
          thisProcessingReport.measurement_delay = 0.;
          thisProcessingReport.observation_available = true;

          const auto [cloudInfo, timestampCloudInfo] = lidarInputBuffer_.get_first_buffer_time_pair_and_pop();

          thisProcessingReport.ts_measurement = timestampCloudInfo.seconds();
          const auto ts_processing_start = node_.now();
          thisProcessingReport.ts_start_processing = ts_processing_start.seconds();

          RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: process lidar frame at: " << std::fixed << timestampCloudInfo.seconds() << " current input size: " << lidarInputSize);
          odomInfoMsg_->header = cloudInfo.header;

          // extract time stamp
          timestampCloudInfo_ = timestampCloudInfo;
          odomInfoMsg_->header = cloudInfo.header;

          // extract info and feature cloud
          currentCloudInfo_ = cloudInfo;
          pcl::fromROSMsg(cloudInfo.cloud_corner, *laserCloudCornerLast_);
          pcl::fromROSMsg(cloudInfo.cloud_surface, *laserCloudSurfLast_);
          // INFORMATION
          // the number of entries in cloudKeyPoseIndexTimestampMap_ must be the same to it of cloudKeyPose3D_ and cloudKeyPose6D_

          //ExcutiveLockGuard lg(mutex_);

          //auto currentPose = setCurrentOdomPrior(timestampCloudInfo_); // abs(currentPosePrior_.timestamp.seconds() - timestampCloudInfo_.seconds());

          const auto queryOutput = this->queryLiDARPoseFromOptPose(timestampCloudInfo_);

          if(!queryOutput.queriedSuccess)
          {
            RCLCPP_ERROR_STREAM(node_.get_logger(), "!!!!!!!!!!!!!!!!! CANT QUERY POSE FOR THE SCAN! " << timestampCloudInfo_.seconds());
            continue;
          }

         const auto currentPose = queryOutput.pose;

          //RCLCPP_ERROR_STREAM(appPtr_.get_logger(), "Querried Initial Pose: " << std::fixed << currentPose );

          // updateKeyCloudPoseImpl_
          transformTobeMapped_[0] = currentPose.rotation().roll();
          transformTobeMapped_[1] = currentPose.rotation().pitch();
          transformTobeMapped_[2] = currentPose.rotation().yaw();
          transformTobeMapped_[3] = currentPose.translation().x();
          transformTobeMapped_[4] = currentPose.translation().y();
          transformTobeMapped_[5] = currentPose.translation().z();

          incrementalOdometryAffineFront_ = trans2Affine3f(transformTobeMapped_);

          // first, we check whether the cloudKeyPose was updated for last scan in last optimization
          // to do so, we just check whether the keyPoseCounter is the same as the size of the cloudKeyPose
          // if it is not the case, it means the fgo was not done and we just skip this scan and notify skippedOptimization
          // so that the next scan can be optimized immediately if the cloudKeyPose was updated

          const auto currentTimeSec = timestampCloudInfo_.seconds();

          skippedOptimization = false;

          RCLCPP_INFO_STREAM(node_.get_logger(), "[LIOSAM]: calculate odom for the SCAN at " << std::fixed << currentTimeSec
                                                                                           << " . Current Map size: " << cloudKeyPoses3D_->size());
          this->extractSurroundingKeyFrames();
          //RCLCPP_WARN(appPtr_.get_logger(), "ON LASER SCAN PROCESSING: Extracted surrounding key frames");

          this->downsampleCurrentScan();
          //RCLCPP_WARN(appPtr_.get_logger(), "ON LASER SCAN PROCESSING: downsampled");

          // MAIN LIDAR ODOM OPTIMAZATION
          const auto [optimization_done, deltaR, deltaT] = this->scan2MapOptimization();
          //RCLCPP_WARN(appPtr_.get_logger(), "ON LASER SCAN PROCESSING: scan matched!");

          thisProcessingReport.residual_information.emplace_back(deltaR);
          thisProcessingReport.residual_information.emplace_back(deltaT);

          scanCounter ++;
          if( !optimization_done) {
              RCLCPP_ERROR_STREAM(node_.get_logger(), " Invalid LIOSAM realtive pose, start again. Optimization done: " << optimization_done );
              //isFirstScan_ = true;
              //last_pose = currentPose.pose;
              this->updateKeyCloudPoseImpl_(keyPoseCounter_,  currentPose, currentTimeSec); // poseToENU   currentPose.pose
              keyPoseCounter_ ++;
              lastQueryStateOutput_ = queryOutput;
              lastOptFinished_ = true;

              thisProcessingReport.duration_processing = (node_.now() - ts_processing_start).seconds();
              thisProcessingReport.observation_available = false;
              thisProcessingReport.message = "Optimization failed, check residual information.";
              thisProcessingReport.header.stamp = node_.now();
              if(pubSensorReport_)
                pubSensorReport_->publish(thisProcessingReport);
              continue;
          }

          incrementalOdometryAffineBack_ = trans2Affine3f(transformTobeMapped_);

          static gtsam::Pose3 lTimu = gtsam::Pose3(params_.RotIMUtoLiDAR, params_.lbIMUtoLiDAR);
          static gtsam::Rot3 imuRl = params_.RotIMUtoLiDAR.inverse();
          // lidar pose in local world frame
          gtsam::Pose3 poseFromLocal = pclPointTogtsamPose3(cloudKeyPoses6D_->points.back());
          gtsam::Pose3 poseToLocal = trans2gtsamPose(transformTobeMapped_);

          const auto relativePose = poseFromLocal.between(poseToLocal);

          odomInfoMsg_->odom_local_pos.x = relativePose.x();
          odomInfoMsg_->odom_local_pos.y = relativePose.y();
          odomInfoMsg_->odom_local_pos.z = relativePose.z();
          odomInfoMsg_->odom_local_rpy.x = relativePose.rotation().roll();
          odomInfoMsg_->odom_local_rpy.y = relativePose.rotation().pitch();
          odomInfoMsg_->odom_local_rpy.z = relativePose.rotation().yaw();

          odomInfoMsg_->sensor_local_pos_from.x = poseFromLocal.x();
          odomInfoMsg_->sensor_local_pos_from.y = poseFromLocal.y();
          odomInfoMsg_->sensor_local_pos_from.z = poseFromLocal.z();
          odomInfoMsg_->sensor_local_rpy_from.x = poseFromLocal.rotation().roll();
          odomInfoMsg_->sensor_local_rpy_from.y = poseFromLocal.rotation().pitch();
          odomInfoMsg_->sensor_local_rpy_from.z = poseFromLocal.rotation().yaw();
          odomInfoMsg_->sensor_local_pos_to.x = poseToLocal.x();
          odomInfoMsg_->sensor_local_pos_to.y = poseToLocal.y();
          odomInfoMsg_->sensor_local_pos_to.z = poseToLocal.z();
          odomInfoMsg_->sensor_local_rpy_to.x = poseToLocal.rotation().roll();
          odomInfoMsg_->sensor_local_rpy_to.y = poseToLocal.rotation().pitch();
          odomInfoMsg_->sensor_local_rpy_to.z = poseToLocal.rotation().yaw();

          odomInfoMsg_->key_current = keyPoseCounter_;
          odomInfoMsg_->key_previous = keyPoseCounter_ - 1;

          const auto positionFromENU = (rotOffsetInitENU_wRn_.inverse()).rotate(poseFromLocal.translation());
          const auto positionToENU = (rotOffsetInitENU_wRn_.inverse()).rotate(poseToLocal.translation());

          odomInfoMsg_->sensor_enu_pos_from.x = positionFromENU.x();
          odomInfoMsg_->sensor_enu_pos_from.y = positionFromENU.y();
          odomInfoMsg_->sensor_enu_pos_from.z = positionFromENU.z();
          odomInfoMsg_->sensor_enu_pos_to.x = positionToENU.x();
          odomInfoMsg_->sensor_enu_pos_to.y = positionToENU.y();
          odomInfoMsg_->sensor_enu_pos_to.z = positionToENU.z();

          const auto positionFromECEF = (rotInitENU_.inverse()).rotate(positionFromENU) + posAnchorInitECEF_;
          const auto positionToECEF = (rotInitENU_.inverse()).rotate(positionToENU) + posAnchorInitECEF_;

          odomInfoMsg_->sensor_ecef_pos_from.x = positionFromECEF.x();
          odomInfoMsg_->sensor_ecef_pos_from.y = positionFromECEF.y();
          odomInfoMsg_->sensor_ecef_pos_from.z = positionFromECEF.z();
          odomInfoMsg_->sensor_ecef_pos_to.x = positionToECEF.x();
          odomInfoMsg_->sensor_ecef_pos_to.y = positionToECEF.y();
          odomInfoMsg_->sensor_ecef_pos_to.z = positionToECEF.z();

          // auto LiDARRotOffsetLocal = yawOffsetRot.compose(rotInitENU_.compose(rotationLiDARECEF));

          const auto rotationFromENU = (rotOffsetInitENU_wRn_.inverse()).compose(poseFromLocal.rotation());
          const auto rotationToENU = (rotOffsetInitENU_wRn_.inverse()).compose(poseToLocal.rotation());

          odomInfoMsg_->sensor_enu_rpy_from.x = rotationFromENU.roll();
          odomInfoMsg_->sensor_enu_rpy_from.y = rotationFromENU.pitch();
          odomInfoMsg_->sensor_enu_rpy_from.z = rotationFromENU.yaw();
          odomInfoMsg_->sensor_enu_rpy_to.x = rotationToENU.roll();
          odomInfoMsg_->sensor_enu_rpy_to.y = rotationToENU.pitch();
          odomInfoMsg_->sensor_enu_rpy_to.z = rotationToENU.yaw();

          const auto rotationFromECEF = (rotInitENU_.inverse()).compose(rotationFromENU);
          const auto rotationToECEF = (rotInitENU_.inverse()).compose(rotationFromENU);

          odomInfoMsg_->sensor_ecef_rpy_from.x = rotationFromECEF.roll();
          odomInfoMsg_->sensor_ecef_rpy_from.y = rotationFromECEF.pitch();
          odomInfoMsg_->sensor_ecef_rpy_from.z = rotationFromECEF.yaw();
          odomInfoMsg_->sensor_ecef_rpy_to.x = rotationToECEF.roll();
          odomInfoMsg_->sensor_ecef_rpy_to.y = rotationToECEF.pitch();
          odomInfoMsg_->sensor_ecef_rpy_to.z = rotationToECEF.yaw();

          const auto poseFromECEF = poseAnchorLocalWorldToECEF_.transformPoseFrom(poseFromLocal);
          //gtsam::Pose3(rotationFromECEF, positionFromECEF);
          const auto poseToECEF = poseAnchorLocalWorldToECEF_.transformPoseFrom(poseToLocal);
          //gtsam::Pose3(rotationToECEF, positionToECEF);

          // should be the same as relativePoseLocal
          const auto relativePoseECEF = poseFromECEF.between(poseToECEF);

          odomInfoMsg_->counter = odomInfoCounter_++;
          odomInfoMsg_->odom_ecef_pos.x = relativePoseECEF.x();
          odomInfoMsg_->odom_ecef_pos.y = relativePoseECEF.y();
          odomInfoMsg_->odom_ecef_pos.z = relativePoseECEF.z();
          odomInfoMsg_->odom_ecef_rpy.x = relativePoseECEF.rotation().roll();
          odomInfoMsg_->odom_ecef_rpy.y = relativePoseECEF.rotation().pitch();
          odomInfoMsg_->odom_ecef_rpy.z = relativePoseECEF.rotation().yaw();

          uint64_t keyPoseCounterTmp = keyPoseCounter_;
          cloudKeyPoseIndexTimestampMap_.insert(std::make_pair(keyPoseCounterTmp, currentTimeSec));

          const auto IMUPoseFromLocal = poseFromLocal.transformPoseFrom(lTimu);
          const auto IMUPoseToLocal = poseToLocal.transformPoseFrom(lTimu);

          fgo::data_types::Odom odom;
          odom.noise = params_.LiDAROdomVariance;
          odom.poseFromLocalWorld = IMUPoseFromLocal;
          odom.poseFromECEF = poseFromECEF.transformPoseFrom(lTimu);
          odom.poseToLocalWorld = IMUPoseToLocal;
          odom.poseToECEF = poseToECEF.transformPoseFrom(lTimu);
          const auto rpyIncreIMU =  imuRl.rotate(relativePose.rotation().rpy());
          const auto posIncreIMU = imuRl.rotate(relativePose.translation());
          odom.poseRelativeECEF = gtsam::Pose3(gtsam::Rot3::RzRyRx(rpyIncreIMU), posIncreIMU); //odom.poseFromECEF.between(odom.poseToECEF);

          const auto poseOdom = odom.poseFromECEF.between(odom.poseToECEF);

          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << " relative pos: " << odom.poseRelativeECEF.translation());
          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << " relative pos: " << poseOdom.translation());
          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << "relative ori: " << odom.poseRelativeECEF.rotation().rpy()* fgo::constants::rad2deg);
          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << "relative ori: " << poseOdom.rotation().rpy() * fgo::constants::rad2deg);


          //odom.poseRelativeECEF = odom.poseFromECEF.between(odom.poseToECEF);
          odom.frameIndexCurrent = keyPoseCounter_;
          odom.frameIndexPrevious = keyPoseCounter_ - 1;
          odom.timestampPrevious = timestampLastKeyPose_;
          odom.timestampCurrent = currentTimeSec;
          odom.queryOutputPrevious = lastQueryStateOutput_;
          odom.queryOutputCurrent = queryOutput;

          odomBuffer_.update_buffer(odom, timestampCloudInfo_);
          const auto IMUPoseTo = imuStateBuffer_.get_buffer(timestampCloudInfo_).state.pose();
          const auto relativeTrue = lastQueryStateOutput_.poseIMUECEF.between(queryOutput.poseIMUECEF);

          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << "relativePoseECEF IMU pos: " << relativeTrue.translation());
          RCLCPP_WARN_STREAM(rclcpp::get_logger("gnss_fgo"), "[LIOSAM] scan " << scanCounter << std::fixed << "relativePoseECEF IMU ori: " << relativeTrue.rotation().rpy() * fgo::constants::rad2deg);

          lastQueryStateOutput_ = queryOutput;
          lastIMUPoseTrue = IMUPoseTo;

          timestampLastKeyPose_ = currentTimeSec;

          thisProcessingReport.duration_processing = (node_.now() - ts_processing_start).seconds();
          thisProcessingReport.header.stamp = node_.now();
          thisProcessingReport.num_measurements = 1;
          if(pubSensorReport_)
              pubSensorReport_->publish(thisProcessingReport);

          keyPoseCounter_ ++;
          last_pose = currentPose;
          lastOptFinished_ = false;
          RCLCPP_WARN(node_.get_logger(), "ON LASER SCAN PROCESSING: Done");
      }
    }

    void LIOSAMOdometry::extractSurroundingKeyFrames() {
      if(cloudKeyPoses3D_->points.empty())
        return;

      // extract nearby
      pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;

      //std::cout << "++++++++++++++++++++++ Size : cloudKeyPoses3D_" << cloudKeyPoses3D_->size() << std::endl;

      // extract all the nearby key poses and downsample them
      kdtreeSurroundingKeyPoses_->setInputCloud(cloudKeyPoses3D_); // create kd-tree
      kdtreeSurroundingKeyPoses_->radiusSearch(cloudKeyPoses3D_->back(),
                                               (double)params_.surroundingKeyframeSearchRadius,
                                               pointSearchInd,
                                               pointSearchSqDis);

      for (int id : pointSearchInd)
      {
        surroundingKeyPoses->push_back(cloudKeyPoses3D_->points[id]);
      }

      //std::cout << "++++++++++++++++++++++ Size : surroundingKeyPoses" << surroundingKeyPoses->size() << std::endl;

      downSizeFilterSurroundingKeyPoses_.setInputCloud(surroundingKeyPoses);
      downSizeFilterSurroundingKeyPoses_.filter(*surroundingKeyPosesDS);
      for(auto& pt : surroundingKeyPosesDS->points)
      {
        kdtreeSurroundingKeyPoses_->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D_->points[pointSearchInd[0]].intensity;
      }

      // also extract some latest keyframes in case the robot rotates in one position
      auto numPoses = cloudKeyPoses3D_->size();
      for(auto i = numPoses - 1; i >= 0; --i)
      {
        if(timestampCloudInfo_.seconds() - cloudKeyPoses6D_->points[i].time < params_.maxPointAge)
          surroundingKeyPosesDS->push_back(cloudKeyPoses3D_->points[i]);
        else
          break;
      }
      //std::cout << "++++++++++++++++++++++ Size : surroundingKeyPosesDS" << surroundingKeyPosesDS->size() << std::endl;

      this->extractCloud(surroundingKeyPosesDS);
    }

    void LIOSAMOdometry::extractCloud(const pcl::PointCloud<PointType>::Ptr &cloudToExtract) {

      // fuse the map
      laserCloudCornerFromMap_->clear();
      laserCloudSurfFromMap_->clear();

      for(size_t i = 0; i < cloudToExtract->size(); ++i)
      {
        if(pointDistance(cloudToExtract->points[i], cloudKeyPoses3D_->back()) > params_.surroundingKeyframeSearchRadius)
          continue;

        auto thisKeyIntensity = (int)cloudToExtract->points[i].intensity;

        if(laserCloudMapContainer_.find(thisKeyIntensity) != laserCloudMapContainer_.end())
        {
          // transformed cloud available
          *laserCloudCornerFromMap_ += laserCloudMapContainer_[thisKeyIntensity].first;
          *laserCloudSurfFromMap_ += laserCloudMapContainer_[thisKeyIntensity].second;
        } else
        {
          pcl::PointCloud<PointType> laserCloudCornerTmp = *transformPointCloud(cornerCloudKeyFrames_[thisKeyIntensity],
                                                                                &cloudKeyPoses6D_->points[thisKeyIntensity]);
          pcl::PointCloud<PointType> laserCloudSurfTmp = *transformPointCloud(surfCloudKeyFrames_[thisKeyIntensity],
                                                                              &cloudKeyPoses6D_->points[thisKeyIntensity]);
          *laserCloudCornerFromMap_ += laserCloudCornerTmp;
          *laserCloudSurfFromMap_ += laserCloudSurfTmp;
          laserCloudMapContainer_[thisKeyIntensity] = std::make_pair(laserCloudCornerTmp, laserCloudSurfTmp);
        }
      }

      // downsample the surrounding corner keyframes (or map)
      downSizeFilterCorner_.setInputCloud(laserCloudCornerFromMap_);
      downSizeFilterCorner_.filter(*laserCloudCornerFromMapDS_);
      laserCloudCornerFromMapDSNum_ = laserCloudCornerFromMapDS_->size();
      // Downsample the surrounding surf key frames (or map)
      downSizeFilterSurf_.setInputCloud(laserCloudSurfFromMap_);
      downSizeFilterSurf_.filter(*laserCloudSurfFromMapDS_);
      laserCloudSurfFromMapDSNum_ = laserCloudSurfFromMapDS_->size();

      //std::cout << "laserCloudCornerFromMapDSNum: " << laserCloudCornerFromMapDSNum_ << std::endl;
      //std::cout << "laserCloudSurfFromMapDSNum: " << laserCloudSurfFromMapDSNum_ << std::endl;

      // clear map cache if too large
      if (laserCloudMapContainer_.size() > params_.maxNumCachedMap)
      {

        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<int> uni(0, params_.maxNumCachedMap - 1);
        auto oversize = laserCloudMapContainer_.size() - params_.maxNumCachedMap;
        std::cout << "CLEAN LIOSAM MAT with oversize: " << oversize << std::endl;
        for (size_t i = 0; i < oversize; ++i) {
          laserCloudMapContainer_.erase(uni(rng));
        }
      }
    }

    void LIOSAMOdometry::extractForLoopClosure() {
      pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
      auto numPoses = cloudKeyPoses3D_->size();

      for(auto i = numPoses - 1; i >= 0; --i)
      {
        if(cloudToExtract->size() <= params_.surroundingKeyframeSize)
          cloudToExtract->push_back(cloudKeyPoses3D_->points[i]);
        else
          break;
      }

      this->extractCloud(cloudToExtract);

    }

    void LIOSAMOdometry::downsampleCurrentScan() {
      // Downsample cloud from current scan
      laserCloudCornerLastDS_->clear();
      downSizeFilterCorner_.setInputCloud(laserCloudCornerLast_);
      downSizeFilterCorner_.filter(*laserCloudCornerLastDS_);
      laserCloudCornerLastDSNum_ = laserCloudCornerLastDS_->size();

      laserCloudSurfLastDS_->clear();
      downSizeFilterSurf_.setInputCloud(laserCloudSurfLast_);
      downSizeFilterSurf_.filter(*laserCloudSurfLastDS_);
      laserCloudSurfLastDSNum_ = laserCloudSurfLastDS_->size();

      //std::cout <<"******* CurrentCloudCournerSize: " << laserCloudCornerLast_->size() << std::endl;
      //std::cout <<"******* CurrentCloudSurfaceSize: " << laserCloudSurfLast_->size() << std::endl;
      //std::cout <<"******* CurrentDSCloudCournerSize: " << laserCloudCornerLastDSNum_<< std::endl;
      //std::cout <<"******* CurrentDSCloudSurfaceSize: " << laserCloudSurfLastDSNum_ << std::endl;
    }

    std::tuple<bool, double, double> LIOSAMOdometry::scan2MapOptimization() {
      bool optimization_done = false;
      double deltaR = 0., deltaT = 0.;
      if(cloudKeyPoses3D_->points.empty())
        return {false, 0., 0.};

      if(laserCloudCornerLastDSNum_ > params_.edgeFeatureMinValidNum && laserCloudSurfLastDSNum_ > params_.surfFeatureMinValidNum)
      {
        kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMapDS_);
        kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMapDS_);

        for(size_t it = 0; it < 50; it++) // LIOParams_.scan2MapOptIteration
        {
          laserCloudOri_->clear();
          coeffSel_->clear();

          this->cornerOptimization();
          //std::cout << "Corner done!" << std::endl;
          this->surfOptimization();
         // std::cout << "surf done!" << std::endl;
          this->combineOptimizationCoeffs();
          const auto [optimization_converged, deltaR_, deltaT_] = this->LiDARLMOptimization(it);
          deltaR = deltaR_;
          deltaT = deltaT_;
          if(optimization_converged)
          {
            //std::cout << "LiDARLMOptimization done!!" << std::endl;
            optimization_done = true;
            std::cout << "*************** optimization_converged ? " << optimization_converged << std::endl;
            break;
          }

        }
      }
      else
        RCLCPP_WARN(node_.get_logger(), "[LIOSAM]: Not enough features! Only %zu edge and %zu planar features available.", laserCloudCornerLastDSNum_, laserCloudSurfLastDSNum_);
      return {optimization_done, deltaR, deltaT};;
    }

    void LIOSAMOdometry::cornerOptimization() {
      this->updatePointAssociateToMap();

#pragma omp parallel for num_threads(8)
      for(size_t i = 0; i < laserCloudCornerLastDSNum_; i++)
      {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS_->points[i];

        this->pointAssociateToMap(&pointOri, &pointSel);

        kdtreeCornerFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (pointSearchSqDis[4] < 1.0) {
          float cx = 0, cy = 0, cz = 0;
          for (int j = 0; j < 5; j++) {
            cx += laserCloudCornerFromMapDS_->points[pointSearchInd[j]].x;
            cy += laserCloudCornerFromMapDS_->points[pointSearchInd[j]].y;
            cz += laserCloudCornerFromMapDS_->points[pointSearchInd[j]].z;
          }
          cx /= 5; cy /= 5;  cz /= 5;

          float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
          for (int j = 0; j < 5; j++) {
            float ax = laserCloudCornerFromMapDS_->points[pointSearchInd[j]].x - cx;
            float ay = laserCloudCornerFromMapDS_->points[pointSearchInd[j]].y - cy;
            float az = laserCloudCornerFromMapDS_->points[pointSearchInd[j]].z - cz;

            a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
            a22 += ay * ay; a23 += ay * az;
            a33 += az * az;
          }
          a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

          matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
          matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
          matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

          cv::eigen(matA1, matD1, matV1);

          if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

            float x0 = pointSel.x;
            float y0 = pointSel.y;
            float z0 = pointSel.z;
            float x1 = cx + 0.1 * matV1.at<float>(0, 0);
            float y1 = cy + 0.1 * matV1.at<float>(0, 1);
            float z1 = cz + 0.1 * matV1.at<float>(0, 2);
            float x2 = cx - 0.1 * matV1.at<float>(0, 0);
            float y2 = cy - 0.1 * matV1.at<float>(0, 1);
            float z2 = cz - 0.1 * matV1.at<float>(0, 2);

            float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                              + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

            float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

            float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                        + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

            float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                         - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

            float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                         + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

            float ld2 = a012 / l12;

            float s = 1. - 0.9 * fabs(ld2);

            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
            coeff.intensity = s * ld2;

            //std::cout << "Corner s: " << s << std::endl;
            if (s > 0.1) {
              laserCloudOriCornerVec_[i] = pointOri;
              coeffSelCornerVec_[i] = coeff;
              laserCloudOriCornerFlag_[i] = true;
            }
          }
        }
      }
    }

    void LIOSAMOdometry::surfOptimization()
    {
      this->updatePointAssociateToMap();

#pragma omp parallel for num_threads(4)
      for (size_t i = 0; i < laserCloudSurfLastDSNum_; i++)
      {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS_->points[i];
        this->pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = laserCloudSurfFromMapDS_->points[pointSearchInd[j]].x;
            matA0(j, 1) = laserCloudSurfFromMapDS_->points[pointSearchInd[j]].y;
            matA0(j, 2) = laserCloudSurfFromMapDS_->points[pointSearchInd[j]].z;
          }

          matX0 = matA0.colPivHouseholderQr().solve(matB0);

          float pa = matX0(0, 0);
          float pb = matX0(1, 0);
          float pc = matX0(2, 0);
          float pd = 1;

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps; pb /= ps; pc /= ps; pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++) {
            if (fabs(pa * laserCloudSurfFromMapDS_->points[pointSearchInd[j]].x +
                     pb * laserCloudSurfFromMapDS_->points[pointSearchInd[j]].y +
                     pc * laserCloudSurfFromMapDS_->points[pointSearchInd[j]].z + pd) > 0.2) {
              planeValid = false;
              break;
            }
          }

          if (planeValid) {
            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                                                      + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.intensity = s * pd2;

            //std::cout << "Surf s: " << s << std::endl;
            if (s > 0.1) {
              laserCloudOriSurfVec_[i] = pointOri;
              coeffSelSurfVec_[i] = coeff;
              laserCloudOriSurfFlag_[i] = true;
            }
          }
        }
      }
    }

    void LIOSAMOdometry::combineOptimizationCoeffs() {
      // combine corner coeffs
      size_t corner_num = 0, surf_num = 0;
      for (size_t i = 0; i < laserCloudCornerLastDSNum_; ++i){
        if (laserCloudOriCornerFlag_[i]){
          laserCloudOri_->push_back(laserCloudOriCornerVec_[i]);
          coeffSel_->push_back(coeffSelCornerVec_[i]);
          corner_num ++;
        }
      }
      // combine surf coeffs
      for (size_t i = 0; i < laserCloudSurfLastDSNum_; ++i){
        if (laserCloudOriSurfFlag_[i]){
          laserCloudOri_->push_back(laserCloudOriSurfVec_[i]);
          coeffSel_->push_back(coeffSelSurfVec_[i]);
          surf_num ++;
        }
      }

      //std::cout << "''''''''''''''''''' Corner coeff num: " << corner_num << " surf: " << surf_num << std::endl;
      // reset flag for next iteration
      std::fill(laserCloudOriCornerFlag_.begin(), laserCloudOriCornerFlag_.end(), false);
      std::fill(laserCloudOriSurfFlag_.begin(), laserCloudOriSurfFlag_.end(), false);
    }

    std::tuple<bool, double, double> LIOSAMOdometry::LiDARLMOptimization(size_t iterCount) {

      // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
      // lidar <- camera      ---     camera <- lidar
      // x = z                ---     x = y
      // y = x                ---     y = z
      // z = y                ---     z = x
      // roll = yaw           ---     roll = pitch
      // pitch = roll         ---     pitch = yaw
      // yaw = pitch          ---     yaw = roll

      // lidar -> camera
      double srx = sin(transformTobeMapped_[1]);
      double crx = cos(transformTobeMapped_[1]);
      double sry = sin(transformTobeMapped_[2]);
      double cry = cos(transformTobeMapped_[2]);
      double srz = sin(transformTobeMapped_[0]);
      double crz = cos(transformTobeMapped_[0]);

      auto laserCloudSelNum = laserCloudOri_->size();

     // std::cout << "********************** LIOSAM OPTIMIZATION: laserCloudOri Size: " << laserCloudSelNum << std::endl;
      if (laserCloudSelNum < 50) {
        return {false, 0., 0.};
      }

      cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

      PointType pointOri, coeff;

      for (size_t i = 0; i < laserCloudSelNum; i++) {
        // lidar -> camera
        pointOri.x = laserCloudOri_->points[i].y;
        pointOri.y = laserCloudOri_->points[i].z;
        pointOri.z = laserCloudOri_->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel_->points[i].y;
        coeff.y = coeffSel_->points[i].z;
        coeff.z = coeffSel_->points[i].x;
        coeff.intensity = coeffSel_->points[i].intensity;
        // in camera
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                     + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x
                       + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // camera -> lidar
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
      }

      cv::transpose(matA, matAt);
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

      if (iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate_ = false;
        float eignThre[6] = {100., 100., 100., 100., 100., 100.};
        for (int i = 5; i >= 0; i--) {
          if (matE.at<float>(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2.at<float>(i, j) = 0;
            }
            isDegenerate_ = true;
          } else {
            break;
          }
        }
        matP_ = matV.inv() * matV2;
      }

     // std::cout << "*************** optimization_degeneration ? " << isDegenerate_ << std::endl;

      if (isDegenerate_)
      {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP_ * matX2;
      }

      if(!isnan(abs(matX.at<float>(0, 0))))
      {
        transformTobeMapped_[0] += matX.at<float>(0, 0);
        //std::cout << "roll is " << matX.at<double>(0, 0) << std::endl;
      }
      else
        std::cout << "roll nan!" << std::endl;

      if(!isnan(abs(matX.at<float>(1, 0))))
      {
        transformTobeMapped_[1] += matX.at<float>(1, 0);
        //std::cout << "pitch is " << matX.at<double>(1, 0) << std::endl;
      }
      else
        std::cout << "pitch nan!" << std::endl;
      if(!isnan(abs(matX.at<float>(2, 0)))) {
        transformTobeMapped_[2] += matX.at<float>(2, 0);
        //std::cout << "yaw is " << matX.at<double>(2, 0) << std::endl;
      }
      else
        std::cout << "yaw nan!" << std::endl;
      if(!isnan(abs(matX.at<float>(3, 0)))) {
        transformTobeMapped_[3] += matX.at<float>(3, 0);
        //std::cout << "x is " << matX.at<double>(3, 0) << std::endl;
      }
      else
        std::cout << "x nan!" << std::endl;
      if(!isnan(abs(matX.at<float>(4, 0)))) {
        transformTobeMapped_[4] += matX.at<float>(4, 0);
        //std::cout << "y is " << matX.at<double>(4, 0) << std::endl;
      }
      else
        std::cout << "y nan!" << std::endl;
      if(!isnan(abs(matX.at<float>(5, 0)))) {
        transformTobeMapped_[5] += matX.at<float>(5, 0);
        //std::cout << "z is " << matX.at<double>(5, 0) << std::endl;
      }
      else
        std::cout << "z nan!" << std::endl;


      double deltaR = sqrt(
          pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
          pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
          pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
      double deltaT = sqrt(
          pow(matX.at<float>(3, 0) * 100, 2) +
          pow(matX.at<float>(4, 0) * 100, 2) +
          pow(matX.at<float>(5, 0) * 100, 2));

     // std::cout << "############## LIDAR OPTIMIUZATION: deltaR: " << deltaR << " DeltaT: " << deltaT << std::endl;
      if (deltaR < 0.05 && deltaT < 0.05) {
        return {true, deltaR, deltaT}; // converged
      }
      return {false, deltaR, deltaT}; // keep optimizing
    }

    void LIOSAMOdometry::loopClosureThread() {
      this->performLoopClosure();
      this->visualizeLoopClosure();
    }

    void LIOSAMOdometry::performLoopClosure(){
      if(cloudKeyPoses3D_->points.empty())
        return;

      mutex_.lock();
      *cloudKeyPoses3DCopied_ = *cloudKeyPoses3D_;
      *cloudKeyPoses6DCopied_ = *cloudKeyPoses6D_;
      mutex_.unlock();

      //find keys
      size_t loopKeyCur, loopKeyPre;

      if(!detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) &&
         !detectLoopClosureExternal(&loopKeyCur, &loopKeyPre))
        return;

      // extract cloud
      pcl::PointCloud<PointType>::Ptr curKeyframeCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr preKeyframeCloud(new pcl::PointCloud<PointType>());

      this->loopFindNearKeyframes(curKeyframeCloud, loopKeyCur, 0);
      this->loopFindNearKeyframes(preKeyframeCloud, loopKeyPre, params_.historyKeyframeSearchNum);

      if(curKeyframeCloud->size() < params_.minSizeCurrentKeyframeCloud || preKeyframeCloud->size() < params_.minSizePreviousKeyframeCloud)
        return;

      if(pubHistoryKeyFrames_->get_subscription_count() != 0)
        publishCloud(pubHistoryKeyFrames_, preKeyframeCloud, timestampCloudInfo_, params_.odometryFrame);

      // ICP
      static pcl::IterativeClosestPoint<PointType , PointType> icp;

      icp.setMaxCorrespondenceDistance(params_.historyKeyframeSearchRadius * 2);
      icp.setMaximumIterations(params_.icpMaxIterations);
      icp.setTransformationEpsilon(params_.icpTransformEpsilon);
      icp.setEuclideanFitnessEpsilon(params_.icpEuclideanFitnessEpsilon);
      icp.setRANSACIterations(params_.icpRANSACIterations);

      // Align clouds
      icp.setInputSource(curKeyframeCloud);
      icp.setInputTarget(preKeyframeCloud);
      pcl::PointCloud<PointType>::Ptr unusedResult(new pcl::PointCloud<PointType>());
      icp.align(*unusedResult);

      if(!icp.hasConverged() || icp.getFitnessScore() > params_.historyKeyframeFitnessScore)
        return;

      if(pubIcpKeyFrames_->get_subscription_count() != 0)
      {
        pcl::PointCloud<PointType>::Ptr closedCloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*curKeyframeCloud, *closedCloud, icp.getFinalTransformation());
        publishCloud(pubIcpKeyFrames_, closedCloud, timestampCloudInfo_, params_.odometryFrame);
      }

      /*
      // GEt pose transformation
      double x, y, z, roll, pitch, yaw;
      Eigen::Affine3d correctionLiDARFrame;
      correctionLiDARFrame = icp.getFinalTransformation();
      // transform from world origin to wrong pose
      Eigen::Affine3d tWrong = pclPointToAffine3f(cloudKeyPoses6DCopied_->points[loopKeyCur]);
      // transform from world origin
      Eigen::Affine3d tCorrect = correctionLiDARFrame * tWrong;
      pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
      gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
      gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6DCopied_->points[loopKeyPre]);

      //buffer closed loop
      fgo::data_types::Odom loopClosure;
      loopClosure.frameIndexCurrent = loopKeyCur;
      loopClosure.frameIndexPrevious = loopKeyPre;
      loopClosure.timestampCurrent = cloudKeyPoseIndexTimestampMap_[loopKeyCur];
      loopClosure.timestampPrevious = cloudKeyPoseIndexTimestampMap_[loopKeyPre];
      loopClosure.poseRelative = poseFrom.between(poseTo);
      auto noiseScore = icp.getFitnessScore();
      loopClosure.noise = (gtsam::Vector6() << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore).finished();
      loopClosureBuffer_.update_buffer(loopClosure, appPtr_.now());
      loopIndexContainer_[loopKeyCur] = loopKeyPre;*/
    }

    bool LIOSAMOdometry::detectLoopClosureDistance(size_t *latestID, size_t *closestID) {

      auto loopKeyCur = cloudKeyPoses3D_->size() - 1;
      int loopKeyPre = -1;

      // check loop constraint added before
      auto it = loopIndexContainer_.find(loopKeyCur);
      if(it != loopIndexContainer_.end())
        return false;

      // find the closed history keyframe
      std::vector<int> pointSearchIndLoop;
      std::vector<float> pointSearchDisLoop;
      kdtreeHistoryKeyPoses_->setInputCloud(cloudKeyPoses3DCopied_);
      kdtreeHistoryKeyPoses_->radiusSearch(cloudKeyPoses3DCopied_->back(),
                                           params_.historyKeyframeSearchRadius,
                                           pointSearchIndLoop,
                                           pointSearchDisLoop,
                                           0);

      for(const auto& id : pointSearchIndLoop)
      {
        if(abs(cloudKeyPoses6DCopied_->points[id].time - timestampCloudInfo_.seconds()) > params_.historyKeyframeSearchTimeDiff)
        {
          loopKeyPre = id;
          break;
        }
      }

      if(loopKeyPre == -1 ||loopKeyCur == loopKeyPre)
        return false;

      *latestID = loopKeyCur;
      *closestID = loopKeyPre;
      return true;
    }

    bool LIOSAMOdometry::detectLoopClosureExternal(size_t *latestID, size_t *closestID) {
      // this function is not used yet, please ignore it
      int loopKeyCur = -1;
      int loopKeyPre = -1;

      ExcutiveLockGuard lock(mutexLoop_);
      if (loopInfoVec_.empty())
        return false;

      double loopTimeCur = loopInfoVec_.front().data[0];
      double loopTimePre = loopInfoVec_.front().data[1];
      loopInfoVec_.pop_front();

      if (abs(loopTimeCur - loopTimePre) < params_.historyKeyframeSearchTimeDiff)
        return false;

      size_t cloudSize = cloudKeyPoses6DCopied_->size();
      if (cloudSize < 2)
        return false;

      // latest key
      loopKeyCur = cloudSize - 1;
      for (int i = cloudSize - 1; i >= 0; --i)
      {
        if (cloudKeyPoses6DCopied_->points[i].time >= loopTimeCur)
          loopKeyCur = round(cloudKeyPoses6DCopied_->points[i].intensity);
        else
          break;
      }

      // previous key
      loopKeyPre = 0;
      for (int i = 0; i < cloudSize; ++i)
      {
        if (cloudKeyPoses6DCopied_->points[i].time <= loopTimePre)
          loopKeyPre = round(cloudKeyPoses6DCopied_->points[i].intensity);
        else
          break;
      }

      if (loopKeyCur == loopKeyPre)
        return false;

      auto it = loopIndexContainer_.find(loopKeyCur);
      if (it != loopIndexContainer_.end())
        return false;

      *latestID = loopKeyCur;
      *closestID = loopKeyPre;

      return true;
    }

    void LIOSAMOdometry::visualizeLoopClosure() {
      if(loopIndexContainer_.empty())
        return;

      visualization_msgs::msg::MarkerArray markerArray;
      // loop node
      visualization_msgs::msg::Marker markerNode;
      markerNode.header.frame_id = params_.odometryFrame;
      markerNode.header.stamp = timestampCloudInfo_;
      markerNode.action = visualization_msgs::msg::Marker::ADD;
      markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      markerNode.ns = "loop_nodes";
      markerNode.id = 0;
      markerNode.pose.orientation.w = 1;
      markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
      markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
      markerNode.color.a = 1;
      // loop edges
      visualization_msgs::msg::Marker markerEdge;
      markerEdge.header.frame_id = params_.odometryFrame;
      markerEdge.header.stamp = timestampCloudInfo_;
      markerEdge.action = visualization_msgs::msg::Marker::ADD;
      markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
      markerEdge.ns = "loop_edges";
      markerEdge.id = 1;
      markerEdge.pose.orientation.w = 1;
      markerEdge.scale.x = 0.1;
      markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
      markerEdge.color.a = 1;

      for(const auto& it : loopIndexContainer_)
      {
        int key_cur = it.first;
        int key_pre = it.second;
        geometry_msgs::msg::Point p;
        p.x = cloudKeyPoses6DCopied_->points[key_cur].x;
        p.y = cloudKeyPoses6DCopied_->points[key_cur].y;
        p.z = cloudKeyPoses6DCopied_->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = cloudKeyPoses6DCopied_->points[key_pre].x;
        p.y = cloudKeyPoses6DCopied_->points[key_pre].y;
        p.z = cloudKeyPoses6DCopied_->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
      }
      markerArray.markers.push_back(markerNode);
      markerArray.markers.push_back(markerEdge);
      pubLoopConstraintEdge_->publish(markerArray);
    }

    void LIOSAMOdometry::visualizeGlobalMapThread() {
      this->publishGlobalMap();
      if(params_.savePCD)
      {
        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving map to pcd files ..." << std::endl;
        params_.savePCDDirectory = std::getenv("HOME") + params_.savePCDDirectory;
        int unused = system((std::string("exec rm -r ") + params_.savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + params_.savePCDDirectory).c_str());
        pcl::io::savePCDFileASCII(params_.savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D_);
        pcl::io::savePCDFileASCII(params_.savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D_);
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D_->size(); i++) {
          *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames_[i],  &cloudKeyPoses6D_->points[i]);
          *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames_[i],    &cloudKeyPoses6D_->points[i]);
          std::cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D_->size() << " ...";
        }
        downSizeFilterCorner_.setInputCloud(globalCornerCloud);
        downSizeFilterCorner_.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileASCII(params_.savePCDDirectory + "cloudCorner.pcd", *globalCornerCloudDS);
        downSizeFilterSurf_.setInputCloud(globalSurfCloud);
        downSizeFilterSurf_.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileASCII(params_.savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        pcl::io::savePCDFileASCII(params_.savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving map to pcd files completed" << std::endl;
      }
    }

    void LIOSAMOdometry::publishGlobalMap() {
      if(!pubLaserCloudSurround_->get_subscription_count() || cloudKeyPoses3D_->points.empty())
        return;

      pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
      pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

      // kd-tree to find near key frames to visualize
      std::vector<int> pointSearchIndGlobalMap;
      std::vector<float> pointSearchSqDisGlobalMap;
      // search near key frames to visualize
      mutex_.lock();
      kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D_);
      kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D_->back(),
                                    params_.globalMapVisualizationSearchRadius,
                                    pointSearchIndGlobalMap,
                                    pointSearchSqDisGlobalMap, 0);
      mutex_.unlock();

      for(const auto& i : pointSearchIndGlobalMap)
        globalMapKeyPoses->emplace_back(cloudKeyPoses3D_->points[i]);

      // downsample near selected key frames
      pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
      downSizeFilterGlobalMapKeyPoses.setLeafSize(params_.globalMapVisualizationPoseDensity,
                                                  params_.globalMapVisualizationPoseDensity,
                                                  params_.globalMapVisualizationPoseDensity); // for global map visualization
      downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
      downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
      for(auto& pt : globalMapKeyPosesDS->points)
      {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = cloudKeyPoses3D_->points[pointSearchIndGlobalMap[0]].intensity;
      }

      // extract visualized and downsampled key frames
      for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D_->back()) > params_.globalMapVisualizationSearchRadius)
          continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames_[thisKeyInd],
                                                    &cloudKeyPoses6D_->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames_[thisKeyInd],
                                                    &cloudKeyPoses6D_->points[thisKeyInd]);
      }
      // downsample visualized points
      pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
      downSizeFilterGlobalMapKeyFrames.setLeafSize(params_.globalMapVisualizationLeafSize,
                                                   params_.globalMapVisualizationLeafSize,
                                                   params_.globalMapVisualizationLeafSize); // for global map visualization
      downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
      downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
      publishCloud(pubLaserCloudSurround_, globalMapKeyFramesDS, timestampCloudInfo_, params_.odometryFrame);
    }




}