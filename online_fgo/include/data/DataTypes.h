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
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#pragma once

#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>

#include "utils/Constants.h"

namespace fgo::data_types{

    typedef std::array<double, 7> UserEstimation_T;

    enum NoiseModel
    {
        GAUSSIAN,
        HUBER,
        CAUCHY,
        DCS,
        Tukey,
        GemanMcClure,
        Welsch

    };

    enum GPModelType
    {
        WNOA,
        WNOJ
    };

    struct Pose
    {
        rclcpp::Time timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        gtsam::Pose3 pose;
        // should be a vector with 6 entries
        gtsam::Matrix66 poseVar = (gtsam::Vector6() <<
                                                    5.0/180 * M_PI, 5.0/180 * M_PI, 5.0/180 * M_PI, 5, 5, 10).finished().asDiagonal();

        std::shared_mutex mutex;

        Pose() = default;
        inline Pose& operator =(const Pose& ori)
        {
          timestamp = ori.timestamp;
          pose = ori.pose;
          poseVar = ori.poseVar;
          return *this;
        }

        Pose(const Pose& ori) {
          timestamp = ori.timestamp;
          pose = ori.pose;
          poseVar = ori.poseVar;
        }
    };

    struct State
    {
        rclcpp::Time timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        gtsam::NavState state{};
        // should be a vector with 6 entries
        gtsam::Matrix66 poseVar = (gtsam::Vector6() <<
                5.0/180 * M_PI, 5.0/180 * M_PI, 5.0/180 * M_PI, 5, 5, 10).finished().asDiagonal();

        gtsam::Matrix33 velVar = gtsam::I_3x3;    // should be a vector with 6 entries, including omega

        gtsam::imuBias::ConstantBias imuBias;
        gtsam::Matrix66 imuBiasVar = (gtsam::Matrix66() << 0.1 * gtsam::I_3x3, gtsam::Z_3x3,
                                      gtsam::Z_3x3, 0.01/180 * M_PI * gtsam::I_3x3).finished(); // should be a vector with 6 entries acc then gyro

        gtsam::Vector2 cbd{};
        // should be a vector with 2 entries
        gtsam::Matrix22 cbdVar = (gtsam::Matrix22() << 1000,0,0,10).finished();

        gtsam::Vector ddIntAmb = gtsam::Vector1(0);
        gtsam::Matrix ddIntAmbVar = gtsam::I_1x1;

        gtsam::Vector3 omega{};
        gtsam::Matrix33 omegaVar = 0.1/180*M_PI * gtsam::I_3x3;

        gtsam::Vector6 accMeasured{};
        //gtsam::Matrix66 accVar = 0.05 * gtsam::I_6x6;

        std::shared_mutex mutex;

        State() = default;


        inline State& operator =(const State& ori)
        {
          timestamp = ori.timestamp;
          state = ori.state;
          poseVar = ori.poseVar;
          velVar = ori.velVar;
          imuBias = ori.imuBias;
          imuBiasVar = ori.imuBiasVar;
          cbd = ori.cbd;
          cbdVar = ori.cbdVar;
          ddIntAmb = ori.ddIntAmb;
          ddIntAmbVar = ori.ddIntAmbVar;
          omega = ori.omega;
          omegaVar = ori.omegaVar;
          accMeasured = ori.accMeasured;
          //accVar = ori.accVar;
          return *this;
        }

        State(const State& ori) {
          timestamp = ori.timestamp;
          state = ori.state;
          poseVar = ori.poseVar;
          velVar = ori.velVar;
          imuBias = ori.imuBias;
          imuBiasVar = ori.imuBiasVar;
          cbd = ori.cbd;
          cbdVar = ori.cbdVar;
          ddIntAmb = ori.ddIntAmb;
          ddIntAmbVar = ori.ddIntAmbVar;
          omega = ori.omega;
          accMeasured = ori.accMeasured;
         // accVar = ori.accVar;
        }
    };

  struct PPS {
        std::atomic_uint_fast64_t counter;
        std::atomic_int_fast64_t localDelay;  // in milliseconds
        rclcpp::Time lastPPSTime;
    };

    struct IMUMeasurement {
        rclcpp::Time timestamp{0};  // timestamp of current imu meas
        double dt{};  // dt between consequent meas
        gtsam::Quaternion AHRSOri{};
        gtsam::Vector9 AHRSOriCov{};
        gtsam::Vector3 accLin{};
        gtsam::Vector9 accLinCov{};
        gtsam::Vector3 accRot{};
        gtsam::Vector9 accRotCov{};
        gtsam::Vector3 gyro{};
        gtsam::Vector9 gyroCov{};
        gtsam::Vector3 mag{};
        gtsam::Vector9 magCov{};
    };

    struct GNSSObs
    {
        uint32_t satId{};
        gtsam::Vector3 satPos{};
        gtsam::Vector3 satVel{};
        //gtsam::Vector3 refSatPos{};
        double pr{};
        double prVar{};
        double prVarRaw{};
        double dr{};
        double drVar{};
        double cp{};
        double cpVar{};
        double cpVarRaw{};
        double el{};
        double cn0{};
        //uint8_t stats{};
        double locktime{};
        //bool useCP = true;
        bool cycleSlip = true;
        //double timeLastSlip = -1.0;
        bool isLOS = true;
    };

    struct RefSat
    {
        uint8_t refSatSVID;
        gtsam::Vector3 refSatPos{};
        gtsam::Vector3 refSatVel{};
    };

    struct GNSSMeasurementEpoch
    {
        double tow{};
        rclcpp::Time timestamp{};
        double delay;
        std::vector<GNSSObs> obs{};
        double timeOffsetGALGPS{};
        bool isGGTOValid = false;
        uint8_t integrityFlag{};
        gtsam::Vector3 basePosRTCM{};
        RefSat refSatGPS{};
        RefSat refSatGAL{};
        uint8_t ddIDXSyncRef;
        uint8_t ddIDXSyncUser;
    };


    enum GNSSSolutionType
    {
        RTKFIX = 1,
        RTKFLOAT  = 2,
        SINGLE  = 3,
        NO_SOLUTION  = 4
    };

    struct PVASolution
    {
        uint16_t wnc;
        double tow{};
        rclcpp::Time timestamp{};
        double delay = 0.;
        GNSSSolutionType type;
        uint8_t error;

        gtsam::Vector3      xyz_ecef;
        gtsam::Vector3      xyz_var;
        gtsam::Vector3      llh;
        gtsam::Vector3      vel_n;
        gtsam::Vector3      vel_ecef;
        gtsam::Vector3      vel_var;
        gtsam::Rot3         rot;
        gtsam::Rot3         rot_ecef;
        gtsam::Rot3         nRe;
        gtsam::Vector3      rot_var;
        double              undulation;
        double              heading;
        double              cog;
        double              heading_ecef;
        double              heading_var;
        double              roll_pitch;
        double              roll_pitch_var;
        double              trk_gnd;
        double              clock_bias;
        double              clock_drift;
        double              clock_bias_var;
        double              clock_drift_var;

        uint8_t             num_sat;
        uint8_t             num_sat_used;
        uint8_t             num_sat_used_l1;
        uint8_t             num_sat_used_multi;
        uint8_t             num_bases;
        uint32_t            reference_id;
        double              correction_age;
        double              solution_age;

        bool                has_rotation_3D = false;
        bool                has_heading;
        bool                has_velocity;
        bool                has_velocity_3D = false;
        bool                has_roll_pitch;
    };

     struct GNSSMeasurement
     {
         // ToDo: 26.03: @Lars: can you adjust this? If it is not done on 28.03, I will take care
         bool hasRTK = false;
         GNSSMeasurementEpoch measMainAnt{};
         bool hasDualAntenna = false;
         GNSSMeasurementEpoch measAuxAnt{};
         bool hasDualAntennaDD = false;
         GNSSMeasurementEpoch measDualAntennaDD{};
         bool hasRTCMDD = false;
         GNSSMeasurementEpoch measRTCMDD{};

         //double tow{};        // timestamp in GPS timing
         //rclcpp::Time timestamp{};  //  timestamp, when the GNSS meas is arrived at online FGO
         // gnss -> pre-processing -> delay (system time + PPS)
         //double delay;
         //std::vector<GNSSObs> mainAntenna{};
         //std::vector<GNSSObs> auxAntenna{};
         //std::vector<GNSSObs> ddRTCM{};
         //std::vector<GNSSObs> ddDualAntenna{};
         //double timeOffsetGALGPS{};
         //bool isGGTOValid = false;
         //uint8_t integrity_flag{};

        // uint refSatIdRTCM{};
         //uint refSatIdAntAux{};
         //gtsam::Vector3 refSatPosRTCM{};
         //gtsam::Vector3 refSatVelRTCM{};
         //gtsam::Vector3 refSatPosAntAux{};
         //gtsam::Vector3 refSatVelAntAux{};
         //gtsam::Vector3 basePosRTCM{};
         //gtsam::Vector3 basePosAntAux{};
         //bool hasDDRTCM = false;
     };

     struct CycleSlipStruct {
         uint32_t satID{};
         int N{}; //count of not sliped
         double md{}; //mean
         double md2{}; //squared mean
         double sd2{}; //sigma
         bool connection{}; //can sat be found in newest gnss meas
         //double timeLastSlip = -1;
         //bool cycleSlip{}; //sliped since last meas
     };

     struct CSDataStruct {
         uint32_t satID{};
         gtsam::Vector3 satPos{};
         //gtsam::Vector3 satRefPos{};
         double cp{};
         double cpVar{};
         double el{};
     };

      struct QueryStateInput
      {
          gtsam::Pose3 pose;
          gtsam::Vector3 vel;
          gtsam::Vector3 omega;
          gtsam::Vector6 acc;
      };

      struct QueryStateOutput
      {
          bool queriedSuccess = false;
          rclcpp::Time timestampCurrent;
          gtsam::Pose3 pose;
          gtsam::Pose3 poseIMUECEF;
          gtsam::Vector3 vel;
          size_t keyIndexI;
          size_t keyIndexJ;
          double timestampI;
          double timestampJ;
          double durationI;
          gtsam::Vector6 accI;
          gtsam::Vector6 accJ;
          bool keySynchronized;
      };

     struct Odom
     {
         size_t frameIndexCurrent;
         size_t frameIndexPrevious;
         double timestampPrevious;
         double timestampCurrent;
         gtsam::Pose3 poseFromLocalWorld;
         gtsam::Pose3 poseFromECEF;
         gtsam::Pose3 poseToLocalWorld;
         gtsam::Pose3 poseToECEF;
         gtsam::Pose3 poseRelativeECEF;
         gtsam::Vector6 noise;
         QueryStateOutput queryOutputPrevious;
         QueryStateOutput queryOutputCurrent;
     };

     struct OdomResult
     {
         size_t frameIndexCurrent;
         size_t frameIndexPrevious;
         double timestampPrevious;
         double timestampCurrent;
         size_t keyIndexII;
         size_t keyIndexIJ;
         size_t keyIndexJI;
         size_t keyIndexJJ;
         double timestampII;
         double timestampIJ;
         double timestampJI;
         double timestampJJ;
         double durationII;
         gtsam::Vector6 accII;
         gtsam::Vector6 accIJ;
         gtsam::Vector6 accJI;
         gtsam::Vector6 accJJ;
         double durationJI;
         bool keyISynchronized;
         bool keyJSynchronized;
         gtsam::Pose3 posePrevious;
         gtsam::Pose3 poseCurrent;
         gtsam::Pose3 posePreviousIMUECEFQueried;
         gtsam::Pose3 poseCurrentIMUECEFQueried;

     };




}