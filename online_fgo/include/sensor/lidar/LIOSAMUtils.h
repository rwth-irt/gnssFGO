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

#ifndef ONLINE_FGO_LIOSAMUTILS_H
#define ONLINE_FGO_LIOSAMUTILS_H
#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

struct PointXYZIRPYTDouble
{
    double x;
    double y;
    double z;
    double intensity;
    double roll;
    double pitch;
    double yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

typedef pcl::PointXYZI PointType;
typedef PointXYZIRPYTDouble  PointTypePose;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x) (float, y, y)
                                      (float, z, z) (float, intensity, intensity)
                                      (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                      (double, time, time));

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYTDouble,
                                  (double, x, x) (double, y, y)
                                      (double, z, z) (double, intensity, intensity)
                                      (double, roll, roll) (double, pitch, pitch) (double, yaw, yaw)
                                      (double, time, time));

namespace sensors::LiDAR::LIOSAM
{
    using ExcutiveLockGuard = std::lock_guard<std::mutex>;
    using SharedLockGuard = std::shared_lock<std::shared_mutex>;

    inline sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
                                               pcl::PointCloud<PointType>::Ptr thisCloud,
                                               rclcpp::Time thisStamp, std::string thisFrame)
    {
      sensor_msgs::msg::PointCloud2 tempCloud;
      pcl::toROSMsg(*thisCloud, tempCloud);
      tempCloud.header.stamp = thisStamp;
      tempCloud.header.frame_id = thisFrame;
      if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
      return tempCloud;
    }

    inline Eigen::Affine3d pclPointToAffine3f(const PointTypePose& thisPoint)
    {
      Eigen::Affine3d d;
      pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw, d);
      return d;
    }

    inline gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& thisPoint)
    {
      return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                          gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    inline Eigen::Affine3d trans2Affine3f(const std::array<double, 6>& transformIn)
    {
      Eigen::Affine3d d;
      pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2], d);
      return d;
    }

    inline gtsam::Pose3 trans2gtsamPose(const std::array<double, 6>& transformIn)
    {
      return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                          gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    inline float pointDistance(const PointType& p)
    {
      return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }

    inline float pointDistance(const PointType& p1, const PointType& p2)
    {
      return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    inline float constraintTransformation(float value, float limit)
    {
      if (value < -limit)
        value = -limit;
      if (value > limit)
        value = limit;
      return value;
    }

    inline PointTypePose trans2PointTypePose(const std::array<double, 6>& transformIn)
    {
      PointTypePose thisPose6D;
      thisPose6D.x = transformIn[3];
      thisPose6D.y = transformIn[4];
      thisPose6D.z = transformIn[5];
      thisPose6D.roll  = transformIn[0];
      thisPose6D.pitch = transformIn[1];
      thisPose6D.yaw   = transformIn[2];
      return thisPose6D;
    }

    inline pcl::PointCloud<PointType>::Ptr transformPointCloud(const pcl::PointCloud<PointType>::Ptr& cloudIn, PointTypePose* transformIn)
    {
      pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

      auto cloudSize = cloudIn->size();
      cloudOut->resize(cloudSize);

      Eigen::Affine3d transCur;
      pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw, transCur);

      #pragma omp parallel for num_threads(4)
      for (int i = 0; i < cloudSize; ++i)
      {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
      }
      return cloudOut;
    }

    enum class SensorType { VELODYNE, OUSTER };
    struct LIOSAMParam
    {
        //Topics
        std::string pointCloudTopic;

        //Frames
        std::string lidarFrame;
        std::string baselinkFrame;
        std::string odometryFrame;
        std::string mapFrame;

        // GPS Settings
        bool useImuHeadingInitialization;
       // bool useGpsElevation;
        //float gpsCovThreshold;
       // float poseCovThreshold;

        // Save pcd
        bool savePCD;
        std::string savePCDDirectory;

        // Lidar Sensor Configuration
        SensorType sensor = SensorType::VELODYNE;
        int N_SCAN;
        int Horizon_SCAN;
        int downsampleRate;
        float lidarMinRange;
        float lidarMaxRange;

        double maxPosePriorLiDARMsgAging = 0.1;  // in sec

        // IMU
        //float imuAccNoise;
        //float imuGyrNoise;
        //float imuAccBiasN;
       // float imuGyrBiasN;
        //float imuGravity;
        float imuRPYWeight;
        //vector<double> extRotV;
        //vector<double> extRPYV;
        //vector<double> extTransV;
        //Eigen::Matrix3d extRot;
        //Eigen::Matrix3d extRPY;
        //Eigen::Vector3d extTrans;
       // Eigen::Quaterniond extQRPY;

        gtsam::Point3 lbIMUtoLiDAR = gtsam::Point3(0.301411, 0, -0.0596); //0.343321
        gtsam::Point3 lbLiDARtoIMU = gtsam::Point3(-0.301411, 0, -0.0596);
        gtsam::Rot3 RotIMUtoLiDAR = gtsam::Rot3().Roll(- M_PI);


        // LOAM
        //float edgeThreshold;
        //float surfThreshold;
        int edgeFeatureMinValidNum;
        int surfFeatureMinValidNum;

        size_t maxNumCachedMap = 1000000;
        double maxPointAge = 10.0;
        size_t scan2MapOptIteration = 30;

        // voxel filter paprams
        //float odometrySurfLeafSize;
        float mappingCornerLeafSize;
        float mappingSurfLeafSize ;

        float z_tollerance;
        float rotation_tollerance;

        gtsam::Vector6 LiDAROdomVariance = (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.09, 0.16, 0.16).finished();

        // CPU Params
        int numberOfCores = 8;
        double mappingProcessInterval = 0.25;

        // Surrounding map
        float surroundingkeyframeAddingDistThreshold;
        float surroundingkeyframeAddingAngleThreshold;
        float surroundingKeyframeDensity;
        float surroundingKeyframeSearchRadius;

        // Loop closure
        bool  loopClosureEnableFlag;
        float loopClosureFrequency;
        int   surroundingKeyframeSize;
        float historyKeyframeSearchRadius;
        float historyKeyframeSearchTimeDiff;
        int   historyKeyframeSearchNum;
        float historyKeyframeFitnessScore;

        int minSizeCurrentKeyframeCloud = 300;
        int minSizePreviousKeyframeCloud = 1000;

        // icp settings
        int icpMaxIterations = 100;
        double icpTransformEpsilon = 1e-6;
        double icpEuclideanFitnessEpsilon = 1e-6;
        int icpRANSACIterations = 0;

        // global map visualization radius
        float globalMapVisualizationSearchRadius;
        float globalMapVisualizationPoseDensity;
        float globalMapVisualizationLeafSize;

        // threading
        int freqLoopDetectionInSec = 5;
        int freqVisualizationInSec = 5;
    };




}


#endif //ONLINE_FGO_LIOSAMUTILS_H
