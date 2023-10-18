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

#ifndef ONLINE_FGO_LIOSAM_H
#define ONLINE_FGO_LIOSAM_H

#pragma once
#include <tuple>
#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lio_sam/msg/cloud_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <irt_nav_msgs/msg/odom_info.hpp>
#include <irt_nav_msgs/msg/sensor_processing_report.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <atomic>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include "sensor/lidar/LIOSAMUtils.h"
#include "data/Buffer.h"
#include "utils/Constants.h"
#include "data/DataTypes.h"
#include "utils/ROSQoS.h"
#include "utils/NavigationTools.h"

#include "solver/FixedLagSmoother.h"
#include "integrator/param/IntegratorParams.h"
#include "integrator/IntegratorBase.h"
#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "model/gp_interpolator/GPWNOAInterpolatorPose3.h"
#include "model/gp_interpolator/GPWNOJInterpolatorPose3.h"


namespace sensors::LiDAR::LIOSAM
{
    class LIOSAMOdometry
    {
        std::string integratorName_;
        rclcpp::Node& node_;

        rclcpp::Time timestampCloudInfo_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

        fgo::integrator::param::IntegratorOdomParamsPtr integratorParamPtr_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubIMUOdometryIncremental_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge_;
        rclcpp::Publisher<irt_nav_msgs::msg::OdomInfo>::SharedPtr pubOdomInfo_;
        rclcpp::Publisher<irt_nav_msgs::msg::SensorProcessingReport>::SharedPtr pubSensorReport_;

        rclcpp::Subscription<lio_sam::msg::CloudInfo>::SharedPtr subCloud_;

        rclcpp::TimerBase::SharedPtr timerLoopDetection_;
        rclcpp::TimerBase::SharedPtr timerMapVisualization_;

        std::shared_ptr<irt_nav_msgs::msg::OdomInfo> odomInfoMsg_;
        uint64_t odomInfoCounter_ = 0;

        LIOSAMParam params_;
        std::mutex mutex_;
        std::mutex mutexLoop_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> imuStateBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::Pose> posePriorECEFBufferTmp_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::Pose> posePriorBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::Odom> odomBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::Odom> loopClosureBuffer_;

        std::shared_mutex mutexKeyPoseIndexTimestampMap_;
        std::map<uint64_t, double> cloudKeyPoseIndexTimestampMap_;
        lio_sam::msg::CloudInfo currentCloudInfo_;
        fgo::buffer::CircularDataBuffer<lio_sam::msg::CloudInfo> lidarInputBuffer_;
        std::shared_ptr<std::thread> odomMainThread_;
        std::atomic_uint64_t keyPoseCounter_ = 1;

        fgo::data_types::QueryStateOutput lastQueryStateOutput_;
        double timestampLastKeyPose_ = -1.;
        double timestampLastScan_ = -1.;

        std::mutex optPoseMutex_;
        std::map<size_t, std::pair<rclcpp::Time, fgo::data_types::QueryStateInput>> optPoseIndexPairMap_;
        fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap currentOptPoseIndexTimestampMap_;
        //fgo::buffer::CircularDataBuffer<gtsam::Vector6> AccBuffer_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolator_;

        bool isDegenerate_ = false;
        std::atomic_bool lastOptFinished_ = true;

        size_t laserCloudCornerFromMapDSNum_ = 0;
        size_t laserCloudSurfFromMapDSNum_ = 0;
        size_t laserCloudCornerLastDSNum_ = 0;
        size_t laserCloudSurfLastDSNum_ = 0;

        std::map<size_t, size_t> loopIndexContainer_; // from new to old
        std::vector<std::pair<int, int>> loopIndexQueue_;
        std::vector<gtsam::Pose3> loopPoseQueue_;
        std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue_;
        std::deque<std_msgs::msg::Float64MultiArray> loopInfoVec_;

        //gtsam::Pose3 poseInitENU_;
        gtsam::Pose3 poseAnchorLocalWorldToECEF_;
        gtsam::Pose3 poseAnchorECEFToLocalWorld_;
        gtsam::Rot3 rotInitENU_;
        gtsam::Rot3 rotOffsetInitENU_wRn_;
        gtsam::Point3 posAnchorInitECEF_;
        double yawOffsetInitENU_ = 0.;
        std::atomic_bool isFirstScan_{};

        rclcpp::Time firstScanTimestamp_;

        nav_msgs::msg::Path globalPath_;
        std::atomic_bool skipScan_ = false;

        Eigen::Affine3d transPointAssociateToMap_;
        Eigen::Affine3d incrementalOdometryAffineFront_;
        Eigen::Affine3d incrementalOdometryAffineBack_;

        cv::Mat matP_;
        std::array<double, 6> transformTobeMapped_{};

        std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames_;
        std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames_;

        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_;
        pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_;
        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DCopied_;
        pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6DCopied_;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_; // corner feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_; // surf feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS_; // downsampled corner featuer set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS_; // downsampled surf featuer set from odoOptimization

        pcl::PointCloud<PointType>::Ptr laserCloudOri_;
        pcl::PointCloud<PointType>::Ptr coeffSel_;

        std::vector<PointType> laserCloudOriCornerVec_; // corner point holder for parallel computation
        std::vector<PointType> coeffSelCornerVec_;
        std::vector<bool> laserCloudOriCornerFlag_;
        std::vector<PointType> laserCloudOriSurfVec_; // surf point holder for parallel computation
        std::vector<PointType> coeffSelSurfVec_;
        std::vector<bool> laserCloudOriSurfFlag_;

        std::map<int, std::pair<::pcl::PointCloud<PointType>, ::pcl::PointCloud<PointType>>> laserCloudMapContainer_;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS_;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS_;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap_;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap_;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses_;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses_;

        pcl::VoxelGrid<PointType> downSizeFilterCorner_;
        pcl::VoxelGrid<PointType> downSizeFilterSurf_;
        pcl::VoxelGrid<PointType> downSizeFilterICP_;
        pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses_; // for surrounding key poses of scan-to-map optimization


    private:

        void processLidarInput();

        void allocateMemory()
        {
          odomInfoMsg_ = std::make_shared<irt_nav_msgs::msg::OdomInfo>();

          cloudKeyPoses3D_.reset(new pcl::PointCloud<PointType>());
          cloudKeyPoses6D_.reset(new pcl::PointCloud<PointTypePose>());
          cloudKeyPoses3DCopied_.reset(new pcl::PointCloud<PointType>());
          cloudKeyPoses6DCopied_.reset(new pcl::PointCloud<PointTypePose>());

          kdtreeSurroundingKeyPoses_.reset(new pcl::KdTreeFLANN<PointType>());
          kdtreeHistoryKeyPoses_.reset(new pcl::KdTreeFLANN<PointType>());

          laserCloudCornerLast_.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
          laserCloudSurfLast_.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
          laserCloudCornerLastDS_.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
          laserCloudSurfLastDS_.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

          laserCloudOri_.reset(new pcl::PointCloud<PointType>());
          coeffSel_.reset(new pcl::PointCloud<PointType>());

          laserCloudOriCornerVec_.resize(params_.N_SCAN * params_.Horizon_SCAN);
          coeffSelCornerVec_.resize(params_.N_SCAN * params_.Horizon_SCAN);
          laserCloudOriCornerFlag_.resize(params_.N_SCAN * params_.Horizon_SCAN);
          laserCloudOriSurfVec_.resize(params_.N_SCAN * params_.Horizon_SCAN);
          coeffSelSurfVec_.resize(params_.N_SCAN * params_.Horizon_SCAN);
          laserCloudOriSurfFlag_.resize(params_.N_SCAN * params_.Horizon_SCAN);

          std::fill(laserCloudOriCornerFlag_.begin(), laserCloudOriCornerFlag_.end(), false);
          std::fill(laserCloudOriSurfFlag_.begin(), laserCloudOriSurfFlag_.end(), false);

          laserCloudCornerFromMap_.reset(new pcl::PointCloud<PointType>());
          laserCloudSurfFromMap_.reset(new pcl::PointCloud<PointType>());
          laserCloudCornerFromMapDS_.reset(new pcl::PointCloud<PointType>());
          laserCloudSurfFromMapDS_.reset(new pcl::PointCloud<PointType>());

          kdtreeCornerFromMap_.reset(new pcl::KdTreeFLANN<PointType>());
          kdtreeSurfFromMap_.reset(new pcl::KdTreeFLANN<PointType>());

          //poseInitENU_ = gtsam::Pose3();

          isFirstScan_ = true;

          matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
          //matP_.setZero();
          //matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
        }

        void extractForLoopClosure();

        void extractSurroundingKeyFrames();

        void extractCloud(const pcl::PointCloud<PointType>::Ptr& cloudToExtract);

        void downsampleCurrentScan();

        std::tuple<bool, double, double> scan2MapOptimization();

        void updatePointAssociateToMap()
        {
          transPointAssociateToMap_ = trans2Affine3f(transformTobeMapped_);
        }

        void pointAssociateToMap(PointType const * const pi, PointType * const po)
        {
          po->x = transPointAssociateToMap_(0,0) * pi->x + transPointAssociateToMap_(0,1) * pi->y + transPointAssociateToMap_(0,2) * pi->z + transPointAssociateToMap_(0,3);
          po->y = transPointAssociateToMap_(1,0) * pi->x + transPointAssociateToMap_(1,1) * pi->y + transPointAssociateToMap_(1,2) * pi->z + transPointAssociateToMap_(1,3);
          po->z = transPointAssociateToMap_(2,0) * pi->x + transPointAssociateToMap_(2,1) * pi->y + transPointAssociateToMap_(2,2) * pi->z + transPointAssociateToMap_(2,3);
          po->intensity = pi->intensity;
        }

        void transformUpdate()
        {
          if (currentCloudInfo_.imu_available)
          {
            if (std::abs(currentCloudInfo_.imu_pitch_init) < 1.4)
            {
              double imuWeight = params_.imuRPYWeight;
              tf2::Quaternion imuQuaternion;
              tf2::Quaternion transformQuaternion;
              double rollMid, pitchMid, yawMid;

              // slerp roll
              transformQuaternion.setRPY(transformTobeMapped_[0], 0, 0);
              imuQuaternion.setRPY(currentCloudInfo_.imu_roll_init, 0, 0);
              tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
              transformTobeMapped_[0] = rollMid;

              // slerp pitch
              transformQuaternion.setRPY(0, transformTobeMapped_[1], 0);
              imuQuaternion.setRPY(0, currentCloudInfo_.imu_pitch_init, 0);
              tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
              transformTobeMapped_[1] = pitchMid;
            }
          }

          //transformTobeMapped_[0] = constraintTransformation(transformTobeMapped_[0], LIOParams_.rotation_tollerance);
          //transformTobeMapped_[1] = constraintTransformation(transformTobeMapped_[1], LIOParams_.rotation_tollerance);
          //transformTobeMapped_[5] = constraintTransformation(transformTobeMapped_[5], LIOParams_.z_tollerance);

        }

        void cornerOptimization();

        void surfOptimization();

        void combineOptimizationCoeffs();

        std::tuple<bool, double, double> LiDARLMOptimization(size_t iterCount);

        void laserCloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msg);

        void loopClosureThread();

        void performLoopClosure();

        bool detectLoopClosureDistance(size_t *latestID, size_t *closestID);

        bool detectLoopClosureExternal(size_t *latestID, size_t *closestID);

        void visualizeLoopClosure();

        void visualizeGlobalMapThread();

        void publishGlobalMap();

        bool checkLiDAROdometry(const gtsam::Pose3& relativePose)
        {
          const auto& trans = relativePose.translation();
          const auto& rot = relativePose.rotation();
          if (//abs(rot.roll())  < LIOParams_.surroundingkeyframeAddingAngleThreshold &&
              //abs(rot.pitch()) < LIOParams_.surroundingkeyframeAddingAngleThreshold &&
              //abs(rot.yaw())   < LIOParams_.surroundingkeyframeAddingAngleThreshold &&
              sqrt(trans.x()*trans.x() + trans.y()*trans.y() + trans.z()*trans.z()) < params_.surroundingkeyframeAddingDistThreshold)
          {
            return false;
            RCLCPP_WARN_STREAM(node_.get_logger(), "[LIOSAM]: check LiDAR odom failed! Relative pose: " << relativePose);
          }

          return true;
        }

        void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
        {
          // extract near keyframes
          nearKeyframes->clear();
          auto cloudSize = cloudKeyPoses6DCopied_->size();
          for (int i = -searchNum; i <= searchNum; ++i)
          {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
              continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames_[keyNear], &cloudKeyPoses6DCopied_->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames_[keyNear],   &cloudKeyPoses6DCopied_->points[keyNear]);
          }

          if (nearKeyframes->empty())
            return;

          // downsample near keyframes
          pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
          downSizeFilterICP_.setInputCloud(nearKeyframes);
          downSizeFilterICP_.filter(*cloud_temp);
          *nearKeyframes = *cloud_temp;
        }

        void updatePath(const PointTypePose& pose_in)
        {
          geometry_msgs::msg::PoseStamped poseStamped;
          poseStamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
          poseStamped.header.frame_id = params_.odometryFrame;
          poseStamped.pose.position.x = pose_in.x;
          poseStamped.pose.position.y = pose_in.y;
          poseStamped.pose.position.z = pose_in.z;
          tf2::Quaternion q;
          q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
          poseStamped.pose.orientation.x = q.x();
          poseStamped.pose.orientation.y = q.y();
          poseStamped.pose.orientation.z = q.z();
          poseStamped.pose.orientation.w = q.w();
          globalPath_.poses.push_back(poseStamped);
        }

        void publishFrames()
        {
          if(cloudKeyPoses3D_->points.empty())
            return;
          // publish key poses
          publishCloud(pubKeyPoses_, cloudKeyPoses3D_, timestampCloudInfo_, params_.odometryFrame);
          // publish surrounding key frames
          publishCloud(pubRecentKeyFrames_, laserCloudSurfFromMapDS_, timestampCloudInfo_, params_.odometryFrame);
          // publish registered high-res raw cloud

          if(pubRecentKeyFrame_->get_subscription_count() != 0)
          {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped_);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS_, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS_, &thisPose6D);
            publishCloud(pubRecentKeyFrame_, cloudOut, timestampCloudInfo_, params_.odometryFrame);
          }

          // publish registered high-res raw cloud
          if(pubCloudRegisteredRaw_->get_subscription_count() != 0)
          {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(currentCloudInfo_.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped_);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw_, cloudOut, timestampCloudInfo_, params_.odometryFrame);
          }

          // publish path
          if(pubPath_->get_subscription_count() != 0)
          {
            globalPath_.header.stamp = timestampCloudInfo_;
            globalPath_.header.frame_id = params_.odometryFrame;
            pubPath_->publish(globalPath_);
          }
        }


        /**
         * Implementation of updating lidar keyframe pose
         * @param keyCloudIndex
         * @param scanPose
         * @param scanTimestamp
         */
        void updateKeyCloudPoseImpl_(size_t keyCloudIndex,
                                     const gtsam::Pose3& scanPose,
                                     const double& scanTimestamp)
        {
          if(keyCloudIndex > cloudKeyPoses3D_->size())
          {
            RCLCPP_WARN_STREAM(node_.get_logger(), "[LIOSAM]: keyIndex " << keyCloudIndex << " doesn't match the keyCloudSize " << cloudKeyPoses3D_->size());
            return;
          }

          // Coordinate system !!!

          PointType thisPose3D;
          PointTypePose thisPose6D;
          thisPose3D.x = scanPose.translation().x();
          thisPose3D.y = scanPose.translation().y();
          thisPose3D.z = scanPose.translation().z();
          thisPose3D.intensity = cloudKeyPoses3D_->size();

          thisPose6D.x = scanPose.translation().x();
          thisPose6D.y = scanPose.translation().y();
          thisPose6D.z = scanPose.translation().z();
          thisPose6D.intensity = thisPose3D.intensity;
          thisPose6D.roll =  scanPose.rotation().roll();
          thisPose6D.pitch =  scanPose.rotation().pitch();
          thisPose6D.yaw =  scanPose.rotation().yaw();
          thisPose6D.time = scanTimestamp;

          //RCLCPP_ERROR_STREAM(node_.get_logger(), "-------------------Updating TransformTobeMapped and CloudKey: keyindex " << keyCloudIndex << " size: " << cloudKeyPoses3D_->size());

          if(keyCloudIndex == cloudKeyPoses3D_->size())
          {
            cloudKeyPoses3D_->emplace_back(thisPose3D);
            cloudKeyPoses6D_->emplace_back(thisPose6D);

            //std::cout << "-------------------ADDING TransformTobeMapped and CloudKey: keyindex" << keyCloudIndex << std::endl;
            transformTobeMapped_[0] = thisPose6D.roll;
            transformTobeMapped_[1] = thisPose6D.pitch;
            transformTobeMapped_[2] = thisPose6D.yaw;
            transformTobeMapped_[3] = thisPose6D.x;
            transformTobeMapped_[4] = thisPose6D.y;
            transformTobeMapped_[5] = thisPose6D.z;

            // save all the received edge and surf points
            pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudCornerLastDS_,  *thisCornerKeyFrame);
            pcl::copyPointCloud(*laserCloudSurfLastDS_,    *thisSurfKeyFrame);

            // save key frame cloud
            cornerCloudKeyFrames_.emplace_back(thisCornerKeyFrame);
            surfCloudKeyFrames_.emplace_back(thisSurfKeyFrame);

            nav_msgs::msg::Odometry laserOdometryROS;
            laserOdometryROS.header.stamp = timestampCloudInfo_;
            laserOdometryROS.header.frame_id = "odom";
            laserOdometryROS.child_frame_id = "odom_mapping";
            laserOdometryROS.pose.pose.position.x = transformTobeMapped_[3];
            laserOdometryROS.pose.pose.position.y = transformTobeMapped_[4];
            laserOdometryROS.pose.pose.position.z = transformTobeMapped_[5];
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(transformTobeMapped_[0], transformTobeMapped_[1], transformTobeMapped_[2]);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdometryROS.pose.pose.orientation = quat_msg;
            pubLaserOdometryGlobal_->publish(laserOdometryROS);

            // Publish TF
            tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]));
            tf2::TimePoint time_point = tf2_ros::fromRclcpp(timestampCloudInfo_);
            tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, "odom");
            geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
            tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
            trans_odom_to_lidar.child_frame_id = params_.lidarFrame;
           // br_->sendTransform(trans_odom_to_lidar);

            this->publishFrames();

          }
          else  // if keyCloudIndex < cloudKeyPoses3D_->size()
          {
            //std::cout << "-------------------Updating TransformTobeMapped and CloudKey: keyindex" << keyCloudIndex << std::endl;
            cloudKeyPoses3D_->points[keyCloudIndex] = thisPose3D;
            cloudKeyPoses6D_->points[keyCloudIndex] = thisPose6D;
          }
          this->updatePath(thisPose6D);
        }

        /**
         * Once lidar point cloud is available, we use this function to query the global pose as prior pose from propagated states
         * @param timestamp timestamp of the lidar scan
         * @return all information about the queried state (lidar pose)
         */
        fgo::data_types::QueryStateOutput queryLiDARPoseFromOptPose(const rclcpp::Time& timestamp)
        {
          fgo::data_types::QueryStateOutput output;
          using namespace fgo::integrator;
          using namespace gtsam::symbol_shorthand;
          ExcutiveLockGuard lg(optPoseMutex_);
          gtsam::Pose3 poseQueried;

          auto syncResult =
                  fgo::integrator::IntegratorBase::findStateForMeasurement(currentOptPoseIndexTimestampMap_,
                                                                           timestamp.seconds(),
                                                                           integratorParamPtr_);

          if(syncResult.status != StateMeasSyncStatus::DROPPED && syncResult.status != StateMeasSyncStatus::CACHED) {
            RCLCPP_ERROR_STREAM(node_.get_logger(),
                                "LIOSAM: for the odom of Scan " << std::fixed << " at: " << timestamp.seconds() << '\n'
                                                                <<
                                                                "found synchronized State I? " << (syncResult.status ==
                                                                                                   StateMeasSyncStatus::SYNCHRONIZED_I ||
                                                                                                   syncResult.status ==
                                                                                                   StateMeasSyncStatus::SYNCHRONIZED_J
                                                                                                   ? "Yes" : "No")
                                                                << '\n' <<
                                                                "II: " << syncResult.keyIndexI << " at " << syncResult.timestampI << '\n' <<
                                                                "IJ: " << syncResult.keyIndexJ << " at " << syncResult.timestampJ << '\n' <<
                                                                "DurationII: " << syncResult.durationFromStateI << '\n');
          }
          output.timestampCurrent = timestamp;
          output.keyIndexI = syncResult.keyIndexI;
          output.keyIndexJ = syncResult.keyIndexJ;
          output.timestampI = syncResult.timestampI;
          output.timestampJ = syncResult.timestampJ;
          output.durationI = syncResult.durationFromStateI;
          output.queriedSuccess = true;
          if(syncResult.status == StateMeasSyncStatus::DROPPED || syncResult.status == StateMeasSyncStatus::CACHED)
          {
            RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: can't find pose for query!");
            output.queriedSuccess = false;
            return output;
          } else if(syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I || syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J)
          {
            //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: query pose synchronized!");
            output.keySynchronized = true;
            if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I){
              poseQueried = optPoseIndexPairMap_[syncResult.keyIndexI].second.pose;
              //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: FOUND synchronized pose for query scan at I: " << std::fixed << timestampI << "\n" << poseQueried);

            } else {
              poseQueried = optPoseIndexPairMap_[syncResult.keyIndexJ].second.pose;
              syncResult.keyIndexI = syncResult.keyIndexJ;
              //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: FOUND synchronized pose for query scan at J: " << std::fixed << timestampJ << "\n" << poseQueried);
            }
          } else if(syncResult.status == StateMeasSyncStatus::INTERPOLATED)
          {
            //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: query pose INTERPOLATED!");
            output.keySynchronized = false;
            auto inputI = optPoseIndexPairMap_[syncResult.keyIndexI].second;
            auto inputJ = optPoseIndexPairMap_[syncResult.keyIndexJ].second;

            bool stateJExist = syncResult.timestampJ < std::numeric_limits<double>::max();

            if(!stateJExist)
            {
              //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: can't interpolate pose for query, stateJ doesn't exist!");
              output.queriedSuccess = false;
              return output;
            }
            const double delta_t = syncResult.timestampJ - syncResult.timestampI;
            if(integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ)
            {
              interpolator_->recalculate(delta_t, syncResult.durationFromStateI, inputI.acc, inputJ.acc);
              output.accI = inputI.acc;
              output.accJ = inputJ.acc;
            }
            else
              interpolator_->recalculate(delta_t, syncResult.durationFromStateI);

            poseQueried = interpolator_->interpolatePose(inputI.pose, inputI.vel, inputI.omega,
                                                         inputJ.pose, inputJ.vel, inputJ.omega);

            //RCLCPP_WARN_STREAM(node_.get_logger(), "LIO: pose interpolated: " << std::fixed << poseQueried );
          }

          static const auto imuTl = gtsam::Pose3(params_.RotIMUtoLiDAR.inverse(), params_.lbLiDARtoIMU);
          const auto poseLiDARECEF = poseQueried.transformPoseFrom(imuTl);

          output.poseIMUECEF = poseQueried;
          odomInfoMsg_->associated_state_timestamp_i = syncResult.timestampI;
          odomInfoMsg_->associated_state_timestamp_j = syncResult.timestampJ;
          odomInfoMsg_->duration_to_i = syncResult.durationFromStateI;
          odomInfoMsg_->associated_with_i = syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I;
          odomInfoMsg_->associated_with_j = syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J;

          if(isFirstScan_)
          {
            //initialPoseSet = true;
            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: Received first SCAN at" << std::fixed << timestamp.seconds() << " ON INITIAL POSE");

            poseAnchorLocalWorldToECEF_ = poseLiDARECEF;
            poseAnchorECEFToLocalWorld_ = poseLiDARECEF.inverse();
            posAnchorInitECEF_ = poseLiDARECEF.translation();

            // rotation ecef to enu
            gtsam::Rot3 nRe(fgo::utils::enuRe_Matrix(posAnchorInitECEF_));
            rotInitENU_ = nRe;
            yawOffsetInitENU_ = nRe.compose(poseLiDARECEF.rotation()).yaw();
            rotOffsetInitENU_wRn_ = nRe.compose(poseLiDARECEF.rotation()).inverse();

            odomInfoMsg_->odom_anchor_ecef_pos.x = posAnchorInitECEF_.x();
            odomInfoMsg_->odom_anchor_ecef_pos.y = posAnchorInitECEF_.y();
            odomInfoMsg_->odom_anchor_ecef_pos.z = posAnchorInitECEF_.z();
            odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.x = nRe.roll();
            odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.y = nRe.pitch();
            odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.z = nRe.yaw();
            odomInfoMsg_->odom_anchor_yaw_offset = yawOffsetInitENU_;

            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: !!!!!!! GOT INITIAL LIDAR POS " << posAnchorInitECEF_);
            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: !!!!!!! GOT INITIAL LIDAR RPY " << rotInitENU_.rpy() * fgo::constants::rad2deg);
            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: !!!!!!! GOT INITIAL LIDAR RPY OFFSET " << rotOffsetInitENU_wRn_.rpy() * fgo::constants::rad2deg);
            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: !!!!!!! GOT INITIAL LIDAR RPY OFFSET MATRIX " << rotOffsetInitENU_wRn_.matrix());
            RCLCPP_ERROR_STREAM(node_.get_logger(), "[LIOSAM]: OFF INITIAL POSE");
            output.pose = gtsam::Pose3();
            return output;
          }
          output.pose = poseAnchorECEFToLocalWorld_.transformPoseFrom(poseLiDARECEF);
          return output;
        }

    public:
        explicit LIOSAMOdometry(rclcpp::Node& node,
                                fgo::integrator::param::IntegratorOdomParamsPtr integratorParamPtr,
                                const rclcpp::CallbackGroup::SharedPtr& groupLiDAROdom,
                                const rclcpp::CallbackGroup::SharedPtr& groupLoopClosure,
                                const rclcpp::CallbackGroup::SharedPtr& groupVisualization,
                                const std::string& integratorName,
                                rclcpp::Publisher<irt_nav_msgs::msg::SensorProcessingReport>::SharedPtr sensorReportPublisher) : node_(node)
        {
          integratorName_ = integratorName;
          integratorParamPtr_ = integratorParamPtr;
          pubSensorReport_ = sensorReportPublisher;

          node_.declare_parameter("OnlineFGO."+ integratorName +".pointCloudTopic", "points");
          node_.get_parameter("OnlineFGO."+ integratorName +".pointCloudTopic", params_.pointCloudTopic);

          node_.declare_parameter("OnlineFGO."+ integratorName +".lidarFrame", "laser_frame");
          node_.get_parameter("OnlineFGO."+ integratorName +".lidarFrame", params_.lidarFrame);

          node_.declare_parameter("OnlineFGO."+ integratorName +".baselinkFrame", "base_link");
          node_.get_parameter("OnlineFGO."+ integratorName +".baselinkFrame", params_.baselinkFrame);

          node_.declare_parameter("OnlineFGO."+ integratorName + ".odometryFrame", "odom");
          node_.get_parameter("OnlineFGO."+ integratorName +".odometryFrame", params_.odometryFrame);

          node_.declare_parameter("OnlineFGO."+ integratorName +".mapFrame", "map");
          node_.get_parameter("OnlineFGO."+ integratorName +".mapFrame", params_.mapFrame);

          node_.declare_parameter("OnlineFGO."+ integratorName +".savePCD", false);
          node_.get_parameter("OnlineFGO."+ integratorName +".savePCD", params_.savePCD);

          node_.declare_parameter("OnlineFGO."+ integratorName +".savePCDDirectory", "~/Documents/LOAM/");
          node_.get_parameter("OnlineFGO."+ integratorName +".savePCDDirectory", params_.savePCDDirectory);

          node_.declare_parameter("OnlineFGO."+ integratorName +".N_SCAN", 16);
          node_.get_parameter("OnlineFGO."+ integratorName +".N_SCAN", params_.N_SCAN);

          node_.declare_parameter("OnlineFGO."+ integratorName +".Horizon_SCAN", 1800);
          node_.get_parameter("OnlineFGO."+ integratorName +".Horizon_SCAN", params_.Horizon_SCAN);

          node_.declare_parameter("OnlineFGO."+ integratorName +".downsampleRate", 1);
          node_.get_parameter("OnlineFGO."+ integratorName +".downsampleRate", params_.downsampleRate);

          node_.declare_parameter("OnlineFGO."+ integratorName +".lidarMinRange", 3.);
          node_.get_parameter("OnlineFGO."+ integratorName +".lidarMinRange", params_.lidarMinRange);

          node_.declare_parameter("OnlineFGO."+ integratorName +".lidarMaxRange", 1000.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".lidarMaxRange", params_.lidarMaxRange);

          node_.declare_parameter("OnlineFGO."+ integratorName +".maxPosePriorLiDARMsgAging", 1000.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".maxPosePriorLiDARMsgAging", params_.maxPosePriorLiDARMsgAging);

          node_.declare_parameter("OnlineFGO."+ integratorName +".edgeFeatureMinValidNum", 10);
          node_.get_parameter("OnlineFGO."+ integratorName +".edgeFeatureMinValidNum", params_.edgeFeatureMinValidNum);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surfFeatureMinValidNum", 100);
          node_.get_parameter("OnlineFGO."+ integratorName +".surfFeatureMinValidNum", params_.surfFeatureMinValidNum);

          node_.declare_parameter("OnlineFGO."+ integratorName +".numberOfCores", 8);
          node_.get_parameter("OnlineFGO."+ integratorName +".numberOfCores", params_.numberOfCores);

          node_.declare_parameter("OnlineFGO."+ integratorName +".mappingProcessInterval", 0.15);
          node_.get_parameter("OnlineFGO."+ integratorName +".mappingProcessInterval", params_.mappingProcessInterval);
          RCLCPP_INFO_STREAM(node_.get_logger(), "LIOSAM PARAM mappingProcessInterval: " << params_.mappingProcessInterval);

          double varRotX = 0.05, varRotY = 0.05, varRotZ = 0.05, varTransX = 0.01, varTransY = 0.09, varTransZ = 0.16;
          node_.declare_parameter("OnlineFGO."+ integratorName +".varianceRoll", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".varianceRoll", varRotX);

          node_.declare_parameter("OnlineFGO."+ integratorName +".variancePitch", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".variancePitch", varRotY);

          node_.declare_parameter("OnlineFGO."+ integratorName +".varianceYaw", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".varianceYaw", varRotZ);

          node_.declare_parameter("OnlineFGO."+ integratorName +".varianceX", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".varianceX", varTransX);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".varianceY", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".varianceY", varTransY);

          node_.declare_parameter("OnlineFGO."+ integratorName +".varianceZ", 0.05);
          node_.get_parameter("OnlineFGO."+ integratorName +".varianceZ", varTransZ);

          params_.LiDAROdomVariance = (gtsam::Vector6() << varRotX, varRotY, varRotZ, varTransX, varTransY, varTransZ).finished();

          RCLCPP_INFO_STREAM(node_.get_logger(), "LIOSAM PARAM odom var: \n" << params_.LiDAROdomVariance);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeSearchRadius", 50.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeSearchRadius", params_.surroundingKeyframeSearchRadius);

          node_.declare_parameter("OnlineFGO."+ integratorName +".loopClosureEnableFlag", true);
          node_.get_parameter("OnlineFGO."+ integratorName +".loopClosureEnableFlag", params_.loopClosureEnableFlag);

          node_.declare_parameter("OnlineFGO."+ integratorName +".loopClosureFrequency", 1.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".loopClosureFrequency", params_.loopClosureFrequency);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeSize", 50); // 50
          node_.get_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeSize", params_.surroundingKeyframeSize);

          node_.declare_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchRadius", 15.0); // 15.
          node_.get_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchRadius", params_.historyKeyframeSearchRadius);

          node_.declare_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchTimeDiff", 30.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchTimeDiff", params_.historyKeyframeSearchTimeDiff);

          node_.declare_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchNum", 30); // 30
          node_.get_parameter("OnlineFGO."+ integratorName +".historyKeyframeSearchNum", params_.historyKeyframeSearchNum);

          node_.declare_parameter("OnlineFGO."+ integratorName +".historyKeyframeFitnessScore", 0.3); // 0.3
          node_.get_parameter("OnlineFGO."+ integratorName +".historyKeyframeFitnessScore", params_.historyKeyframeFitnessScore);

          node_.declare_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationSearchRadius", 1000.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationSearchRadius", params_.globalMapVisualizationSearchRadius);

          node_.declare_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationPoseDensity", 10.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationPoseDensity", params_.globalMapVisualizationPoseDensity);

          node_.declare_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationLeafSize", 1.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".globalMapVisualizationLeafSize", params_.globalMapVisualizationLeafSize);

          node_.declare_parameter("OnlineFGO."+ integratorName +".imuRPYWeight", 0.01);
          node_.get_parameter("OnlineFGO."+ integratorName +".imuRPYWeight", params_.imuRPYWeight);

          node_.declare_parameter("OnlineFGO."+ integratorName +".z_tollerance", 1000.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".z_tollerance", params_.z_tollerance);

          node_.declare_parameter("OnlineFGO."+ integratorName +".rotation_tollerance", 1000.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".rotation_tollerance", params_.rotation_tollerance);

          node_.declare_parameter("OnlineFGO."+ integratorName +".mappingCornerLeafSize", 0.2);
          node_.get_parameter("OnlineFGO."+ integratorName +".mappingCornerLeafSize", params_.mappingCornerLeafSize);

          node_.declare_parameter("OnlineFGO."+ integratorName +".mappingSurfLeafSize", 0.4);
          node_.get_parameter("OnlineFGO."+ integratorName +".mappingSurfLeafSize", params_.mappingSurfLeafSize);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surroundingkeyframeAddingDistThreshold", 0.2);
          node_.get_parameter("OnlineFGO."+ integratorName +".surroundingkeyframeAddingDistThreshold", params_.surroundingkeyframeAddingDistThreshold);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surroundingkeyframeAddingAngleThreshold", 0.2);
          node_.get_parameter("OnlineFGO."+ integratorName +".surroundingkeyframeAddingAngleThreshold", params_.surroundingkeyframeAddingAngleThreshold);

          node_.declare_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeDensity", 2.0);
          node_.get_parameter("OnlineFGO."+ integratorName +".surroundingKeyframeDensity", params_.surroundingKeyframeDensity);


          params_.RotIMUtoLiDAR = integratorParamPtr_->rotIMUtoLiDAR;
          params_.lbLiDARtoIMU = integratorParamPtr_->transIMUToLiDAR;
          params_.lbIMUtoLiDAR = -params_.lbLiDARtoIMU; //LIOParams_.RotIMUtoLiDAR.unrotate(LIOParams_.lbIMUtoLiDAR);

          std::cout << "[LIO:] rot lidarRimu: " << params_.RotIMUtoLiDAR << std::endl;
          std::cout << "[LIO:] trans lbIMUtoLiDAR: " << params_.lbIMUtoLiDAR << std::endl;
          std::cout << "[LIO:] trans lbLiDARtoIMU: " << params_.lbLiDARtoIMU << std::endl;

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".maxNumCachedMap", 1000000);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".maxNumCachedMap", params_.maxNumCachedMap);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".maxPointAge", 10.);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".maxPointAge", params_.maxPointAge);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".scan2MapOptIteration", 30);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".scan2MapOptIteration", params_.scan2MapOptIteration);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".minSizeCurrentKeyframeCloud", 300);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".minSizeCurrentKeyframeCloud", params_.minSizeCurrentKeyframeCloud);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".minSizePreviousKeyframeCloud", 1000);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".minSizePreviousKeyframeCloud", params_.minSizePreviousKeyframeCloud);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".icpMaxIterations", 100);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".icpMaxIterations", params_.icpMaxIterations);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".icpTransformEpsilon", 1e-6);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".icpTransformEpsilon", params_.icpTransformEpsilon);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".icpEuclideanFitnessEpsilon", 1e-6);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".icpEuclideanFitnessEpsilon", params_.icpEuclideanFitnessEpsilon);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".icpRANSACIterations", 0);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".icpRANSACIterations", params_.icpRANSACIterations);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".freqLoopDetectionInSec", 5);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".freqLoopDetectionInSec", params_.freqLoopDetectionInSec);

          node_.declare_parameter("OnlineFGO.\"+ integratorName +\".freqVisualizationInSec", 5);
          node_.get_parameter("OnlineFGO.\"+ integratorName +\".freqVisualizationInSec", params_.freqVisualizationInSec);

          pubLaserCloudSurround_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/map_global", 1);
          pubKeyPoses_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/trajectory", 1);
          pubPath_ = node_.create_publisher<nav_msgs::msg::Path>("/gnss_fgo/mapping/path", 1);
          pubHistoryKeyFrames_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/icp_loop_closure_history_cloud", 1);
          pubLaserOdometryGlobal_ = node_.create_publisher<nav_msgs::msg::Odometry>("/gnss_fgo/mapping/odometry", utils::ros::QoSGeneral);
          pubLaserOdometryIncremental_ = node_.create_publisher<nav_msgs::msg::Odometry>("/gnss_fgo/mapping/odometry_incremental", utils::ros::QoSGeneral);
          pubIcpKeyFrames_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/icp_loop_closure_history_cloud", 1);
          pubLoopConstraintEdge_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>("/gnss_fgo/mapping/loop_closure_constraints", 1);
          pubRecentKeyFrames_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/map_local", 1);;
          pubRecentKeyFrame_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/cloud_registered", 1);;
          pubCloudRegisteredRaw_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/gnss_fgo/mapping/cloud_registered_raw", 1);;
          pubOdomInfo_ = node_.create_publisher<irt_nav_msgs::msg::OdomInfo>("/gnss_fgo/odom_info", 1);
          pubIMUOdometryIncremental_ = pubLaserOdometryIncremental_ = node_.create_publisher<nav_msgs::msg::Odometry>("/gnss_fgo/imu/odometry_incremental", utils::ros::QoSGeneral);

          downSizeFilterCorner_.setLeafSize(params_.mappingCornerLeafSize, params_.mappingCornerLeafSize, params_.mappingCornerLeafSize);
          downSizeFilterSurf_.setLeafSize(params_.mappingSurfLeafSize, params_.mappingSurfLeafSize, params_.mappingSurfLeafSize);
          downSizeFilterICP_.setLeafSize(params_.mappingSurfLeafSize, params_.mappingSurfLeafSize, params_.mappingSurfLeafSize);
          downSizeFilterSurroundingKeyPoses_.setLeafSize(params_.surroundingKeyframeDensity, params_.surroundingKeyframeDensity, params_.surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

          br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

          this->allocateMemory();

          odomBuffer_.resize_buffer(1000);
          loopClosureBuffer_.resize_buffer(10);
          posePriorBuffer_.resize_buffer(1000);
          posePriorECEFBufferTmp_.resize_buffer(1000);
          imuStateBuffer_.resize_buffer(1000);
          lidarInputBuffer_.resize_buffer(50);
          auto subCloudOpt = rclcpp::SubscriptionOptions();
          subCloudOpt.callback_group = groupLiDAROdom;
          subCloud_ = node_.create_subscription<lio_sam::msg::CloudInfo>("/lio_sam/feature/cloud_info",
                                                                         utils::ros::QoSGeneral,
                                                                         [this](const lio_sam::msg::CloudInfo::SharedPtr msg)->void
                                                                         {
                                                                             this->laserCloudInfoHandler(msg);
                                                                         },
                                                                         subCloudOpt);
          if(0 && params_.loopClosureEnableFlag)
          {
            timerLoopDetection_ = node_.create_wall_timer(std::chrono::seconds(params_.freqLoopDetectionInSec),
                                                          [this]()->void
                                                          {
                                                              this->loopClosureThread();
                                                          },
                                                          groupLoopClosure);
          }

          timerMapVisualization_ = node_.create_wall_timer(std::chrono::seconds(params_.freqVisualizationInSec),
                                                        [this]()->void
                                                        {
            this->visualizeGlobalMapThread();
                                                        },
                                                        groupVisualization);

          if(integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ){
            interpolator_ = std::make_shared<fgo::models::GPWNOJInterpolatorPose3>(
                gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
          } else if(integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOA) {
            interpolator_ = std::make_shared<fgo::models::GPWNOAInterpolatorPose3>(
                gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
          } else {
            RCLCPP_WARN(node_.get_logger(), "NO gpType chosen. Please choose.");
          }

          odomMainThread_ = std::make_shared<std::thread>(
              [this]()->void
              {
                this->processLidarInput();
              }
              );

        }

        ~LIOSAMOdometry()
        {
          odomMainThread_->join();
        }

        /**
         * update current global propagated pose for pose querying point cloud callback
         * @param id id of the state, equals to nState_
         * @param timestamp timestamp pf the state id
         * @param input all information about the state, see struct definition
         */
        void updateOptPose(size_t id, const rclcpp::Time& timestamp, const fgo::data_types::QueryStateInput& input)
        {
          ExcutiveLockGuard lg(optPoseMutex_);
          auto iter = optPoseIndexPairMap_.find(id);
          if(iter == optPoseIndexPairMap_.end())
          {
            optPoseIndexPairMap_.insert(std::make_pair(id, std::make_pair(timestamp, input)));
          }
        }

        void updateKeyIndexTimestampMap(const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& currentKeyIndexTimestampMap)
        {
          currentOptPoseIndexTimestampMap_ = currentKeyIndexTimestampMap;
          auto optPoseMapIter = optPoseIndexPairMap_.begin();
          while(optPoseMapIter != optPoseIndexPairMap_.end())
          {
            auto currentIndexIter = currentOptPoseIndexTimestampMap_.find(optPoseMapIter->first);
            if(currentIndexIter == currentOptPoseIndexTimestampMap_.end()) {
             // RCLCPP_WARN_STREAM(node_.get_logger(), "LIOSAM: Erase Pose " << std::fixed << optPoseMapIter->first << " at time: " << optPoseMapIter->second.first.seconds());
              optPoseMapIter = optPoseIndexPairMap_.erase(optPoseMapIter);
              continue;
            }
            optPoseMapIter ++;
          }
        }

        /**
         * update the pose of lidar keyframe after factor graph optimization
         * TODO: @Haoming, maybe extending for all past keyframes in the lag
         * @param scanIndexI scan index of previous frame
         * @param imuPoseI pose of previous state (imu center) that has the same timestamp as scan I
         * @param scanIndexJ scan index of current frame
         * @param imuPoseJ  pose of current state (imu center) that has the same timestamp as scan J
         * @param odomMsg odom message pointer for some statistics
         * @param loopClosed @Haoming, move this back to FGO
         */
        void updateCloudKeyPose(size_t scanIndexI, const gtsam::Pose3& imuPoseI,
                                size_t scanIndexJ, const gtsam::Pose3& imuPoseJ,
                                boost::optional<nav_msgs::msg::Odometry&> odomMsg = boost::none,
                                bool loopClosed = false)
        {
          static const auto imuTl = gtsam::Pose3(params_.RotIMUtoLiDAR.inverse(), params_.lbLiDARtoIMU);
          ExcutiveLockGuard lg(mutex_);

          if(scanIndexI != 0) {
            //auto LiDARPosInLocalTEST = rotInitENU_.matrix() * (positionLiDARECEF - posAnchorInitECEF_);
            //const auto IMUPosIENU = rotInitENU_.rotate(imuPoseI.translation() - posAnchorInitECEF_);
            //const auto IMUPosILocal = rotOffsetInitENU_wRn_.rotate(IMUPosIENU);
            //const auto IMURotILocal = rotOffsetInitENU_wRn_.compose(rotInitENU_.compose(imuPoseI.rotation()));
            //const auto IMUPoseILocal = gtsam::Pose3(IMURotILocal, IMUPosILocal);
            const auto LiDARPoseECEF = imuPoseI.transformPoseFrom(imuTl);
            //const auto LiDARPoseILocal = IMUPoseILocal.transformPoseFrom(imuTl);
            const auto LiDARPoseILocal = poseAnchorECEFToLocalWorld_.transformPoseFrom(LiDARPoseECEF);

            this->updateKeyCloudPoseImpl_(scanIndexI, LiDARPoseILocal, cloudKeyPoseIndexTimestampMap_[scanIndexI]);

            //RCLCPP_ERROR_STREAM(node_.get_logger(),
            //                    "-------------- Updating ScanI " << std::fixed << scanIndexI << " with pose: "
            //                                                     << LiDARPoseILocal);
          }

          //const auto IMUPosJENU = rotInitENU_.rotate(imuPoseJ.translation() - posAnchorInitECEF_);
          //const auto IMUPosJLocal = rotOffsetInitENU_wRn_.rotate(IMUPosJENU);
          //const auto IMURotJLocal = rotOffsetInitENU_wRn_.compose(rotInitENU_.compose(imuPoseJ.rotation()));
          //const auto IMUPoseJLocal = gtsam::Pose3(IMURotJLocal, IMUPosJLocal);
          const auto LiDARPoseECEF = imuPoseJ.transformPoseFrom(imuTl);
          const auto LiDARPoseJLocal = poseAnchorECEFToLocalWorld_.transformPoseFrom(LiDARPoseECEF);

          this->updateKeyCloudPoseImpl_(scanIndexJ, LiDARPoseJLocal, cloudKeyPoseIndexTimestampMap_[scanIndexJ]);
          //RCLCPP_ERROR_STREAM(node_.get_logger(), "-------------- Updating ScanJ " << std::fixed << scanIndexJ << " with pose: " << LiDARPoseJLocal);

          const auto LiDARPoseEcef = imuPoseJ.transformPoseFrom(imuTl);
          odomInfoMsg_->sensor_ecef_pos_to_optimized.x = LiDARPoseEcef.x();
          odomInfoMsg_->sensor_ecef_pos_to_optimized.y = LiDARPoseEcef.y();
          odomInfoMsg_->sensor_ecef_pos_to_optimized.z = LiDARPoseEcef.z();
          odomInfoMsg_->sensor_ecef_rpy_to_optimized.x = LiDARPoseEcef.rotation().roll();
          odomInfoMsg_->sensor_ecef_rpy_to_optimized.y = LiDARPoseEcef.rotation().pitch();
          odomInfoMsg_->sensor_ecef_rpy_to_optimized.z = LiDARPoseEcef.rotation().yaw();

          const auto LiDARPoseENU = gtsam::Pose3(rotOffsetInitENU_wRn_.inverse().compose(LiDARPoseJLocal.rotation()), rotOffsetInitENU_wRn_.unrotate(LiDARPoseJLocal.translation()));
          odomInfoMsg_->sensor_enu_pos_to_optimized.x = LiDARPoseENU.x();
          odomInfoMsg_->sensor_enu_pos_to_optimized.y = LiDARPoseENU.y();
          odomInfoMsg_->sensor_enu_pos_to_optimized.z = LiDARPoseENU.z();
          odomInfoMsg_->sensor_enu_rpy_to_optimized.x = LiDARPoseENU.rotation().roll() * fgo::constants::rad2deg;
          odomInfoMsg_->sensor_enu_rpy_to_optimized.y = LiDARPoseENU.rotation().pitch() * fgo::constants::rad2deg;
          odomInfoMsg_->sensor_enu_rpy_to_optimized.z = LiDARPoseENU.rotation().yaw() * fgo::constants::rad2deg;

          odomInfoMsg_->sensor_local_pos_to_optimized.x = LiDARPoseJLocal.x();
          odomInfoMsg_->sensor_local_pos_to_optimized.y = LiDARPoseJLocal.y();
          odomInfoMsg_->sensor_local_pos_to_optimized.z = LiDARPoseJLocal.z();
          odomInfoMsg_->sensor_local_rpy_to_optimized.x = LiDARPoseJLocal.rotation().roll() * fgo::constants::rad2deg;
          odomInfoMsg_->sensor_local_rpy_to_optimized.y = LiDARPoseJLocal.rotation().pitch() * fgo::constants::rad2deg;
          odomInfoMsg_->sensor_local_rpy_to_optimized.z = LiDARPoseJLocal.rotation().yaw() * fgo::constants::rad2deg;

          pubOdomInfo_->publish(*odomInfoMsg_);
          //if(scanIndexJ == cloudKeyPoseIndexTimestampMap_.end()->first)
          lastOptFinished_ = true;
        }

        bool hasOdom()
        {
          return odomBuffer_.size() > 0;
        }

        void cleanBuffer()
        {
          if(odomBuffer_.size())
            lastOptFinished_ = true;
          odomBuffer_.clean();
          lidarInputBuffer_.clean();
          skipScan_ = true;
        }

        /**
         * Notify intern odometry LS optimization (scan registration)
         */
        void notifyOptimization()
        {
          skipScan_ = false;
          lastOptFinished_ = true;
        }

        void retractKeyPoseCounter(uint64_t step)
        {
          keyPoseCounter_ -= step;
        }

        std::vector<fgo::data_types::Odom> getOdomAndClean()
        {
          std::vector<fgo::data_types::Odom> odom = odomBuffer_.get_all_buffer_and_clean();
          return odom;
        }

        std::vector<fgo::data_types::Odom> getLoopClosureAndClean()
        {
          std::vector<fgo::data_types::Odom> odom = loopClosureBuffer_.get_all_buffer_and_clean();
          return odom;
        }

    };
}
#endif //ONLINE_FGO_LIOSAM_H
