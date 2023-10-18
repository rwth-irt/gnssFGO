//
// Created by haoming on 13.06.22.
//

#include "graphs/GraphGNSSINSIntegration.h"

#include <utility>


namespace fgo::graph
{
    void GraphGNSSINSIntegration::initGraph(const data_types::State &initState,
                                            const double &initTimestamp,
                                            boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams) {



    }

    bool GraphGNSSINSIntegration::constructFactorGraphOnGNSS(std::vector<fgo::data_types::IMUMeasurement>& dataIMU,
                                                             std::vector<fgo::data_types::GNSSMeasurement>& dataGNSS,
                                                             fgo::data_types::State& current_pred_state) {
      //TODO maybe weird sizes, not influenced by actual parameters, because static, dont know how to effectively do
      static boost::circular_buffer<std::pair<size_t, double>> varIDTimestampMap(2.0 * graphParams_.smootherLag * graphParams_.optFrequency);
      static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(2.0 * graphParams_.IMUMeasurementFrequency / graphParams_.optFrequency);

      //RCLCPP_INFO_STREAM(this->get_logger(), "Timestamp LastGNSS: " << std::fixed << gnssDataBuffer_.get_last_buffer().measMainAnt.timestamp.seconds());
      //RCLCPP_INFO_STREAM(this->get_logger(), "Corrected Time of GNSS: " << std::fixed << (gnssDataBuffer_.get_last_buffer().measMainAnt.timestamp.seconds() - gnssDataBuffer_.get_last_buffer().measMainAnt.delay) << "Delay: " << gnssDataBuffer_.get_last_buffer().measMainAnt.delay);

      //RCLCPP_INFO_STREAM(this->get_logger(), "GetBuffer: " << std::fixed << this->get_clock()->now().seconds());
      if ( dataIMU.empty() || dataGNSS.empty() || !isStateInited_ ){
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "No IMU or no GNSS data.");
        return false;
      }
      //RCLCPP_INFO_STREAM(this->get_logger(), "Start constructFactorGraphOnGNSS:...gnssSize: " << dataGNSS.size() << " and IMUSize: " << dataIMU.size() << " and OldIMUSize: "  << dataIMURest_.size());
      //RCLCPP_INFO_STREAM(this->get_logger(), "Delta GNSS: " << std::fixed << dataGNSS.back().measMainAnt.tow << " Delta IMU:" << dataIMU.back().timestamp.seconds() - lastIMUTime.seconds());

      preIntegratorParams_->n_gravity = /*fgo::utils::nedRe_Matrix(lastOptimizedState_.state.position()) * */
          fgo::utils::gravity_ecef(current_pred_state.state.position());
      boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
          boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                       current_pred_state.imuBias);

      //gtsam::Vector3 omega_i = lastOptimizedState_.omega; //needed for GP factor last omega, but no GP here

      //imuPreIntegrationOPT_.
      if (!dataIMURest_.empty()) {
        dataIMU.insert(dataIMU.begin(),dataIMURest_.begin(),dataIMURest_.end());
        dataIMURest_.clear();
      }
      //imuPreIntegrationOPT_->print();
      //preIntegratorParams_->print();

      gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, ddAmb_key_j, omega_key_j,
                 pose_key_i, vel_key_i, bias_key_i, cbd_key_i, ddAmb_key_i, omega_key_i;

      auto first_imu_meas_timestamp = dataIMU.front().timestamp.seconds();  // in double
      auto imu_meas_iter = dataIMU.begin();
      for (auto &meas_gnss: dataGNSS) {
        // firstly, we check, whether there are gnss measurements, which are delayed and should be integrated in LAST optimization epochs!
        //if (meas_gnss.measMainAnt.delay > 0.08)
        //  meas_gnss.measMainAnt.delay = 0.08;
        const double corrected_time_gnss_meas = meas_gnss.measAuxAnt.timestamp.seconds() - meas_gnss.measAuxAnt.delay;   // in double
        //RCLCPP_INFO_STREAM(this->get_logger(), "Now is: "  << std::fixed  << this->get_clock()->now().seconds()) ;
        //RCLCPP_INFO_STREAM(this->get_logger(), "Size of IMU: " << dataIMU.size() << " from: " << imuDataBuffer_.buffer_size) ;
        //RCLCPP_INFO_STREAM(this->get_logger(), "TOW: " << cout.precision(10) << meas_gnss.measMainAnt.tow);
        //RCLCPP_INFO_STREAM(this->get_logger(), "Size dataGNSS: " << std::fixed << dataGNSS.size());

        //RCLCPP_INFO_STREAM(this->get_logger(), "Timestamp GNSS: " << std::fixed << meas_gnss.measMainAnt.timestamp.seconds());
        //RCLCPP_INFO_STREAM(this->get_logger(), "DELAY: " << meas_gnss.measMainAnt.delay);
        //RCLCPP_INFO_STREAM(this->get_logger(), "FIRSTIMU: " << std::fixed << first_imu_meas_timestamp);
        //RCLCPP_INFO_STREAM(this->get_logger(), "LASTIMU: " << std::fixed << dataIMU.back().timestamp.seconds());
        //RCLCPP_INFO_STREAM(this->get_logger(), "Time-DELAY-IMU: " << corrected_time_gnss_meas - first_imu_meas_timestamp);

        if ( corrected_time_gnss_meas < first_imu_meas_timestamp ) {
          RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "[EarlyObs.] found in [onGNSS]");
          RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "STILL TOO LATE GNSS TBD");
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "Delay: " << meas_gnss.measMainAnt.delay);
          dataIMURest_ = dataIMU;
          return false; //TODO for now

          // then we need to find out the gyroscope measurement
          gtsam::Vector3 this_gyro;
          bool find_gyro = false;
          auto timeGyroIter = timeGyroMap.end();
          while (timeGyroIter != timeGyroMap.begin()) {
            if (abs(timeGyroIter->first - corrected_time_gnss_meas) < graphParams_.IMUGNSSSyncTimeThreshold) {
              find_gyro = true;
              this_gyro = timeGyroIter->second;
              break;
            }
            timeGyroIter--;
          }
          // in this case, this meas_gnss is in front of the imu measurement, we could just ignore or interpolate it
          if (graphParams_.UseGPInterpolatedGNSSFactor) {
            RCLCPP_DEBUG(rclcpp::get_logger(rosLoggerName_),
                         "[EarlyObs.] GNSS Obs. in front of IMU measurements, interpolating ...");
            size_t key_i = 0, key_j = 0;
            double key_i_time, key_j_time;
            auto varID_iter = varIDTimestampMap.end();
            while (varID_iter != varIDTimestampMap.begin()) {
              if (varID_iter->second <= corrected_time_gnss_meas) {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(rosLoggerName_),
                                    "[EarlyObs.] Find corresponding i " << varID_iter->first <<
                                                                        " at: " << varID_iter->second);
                // if one state variable was created earlier as this gnss measurement, this is out i
                key_i = varID_iter->first;
                key_i_time = varID_iter->second;
                varID_iter++;
                key_j = varID_iter->first;
                key_j_time = varID_iter->second;
                break;
              }
              varID_iter--;
            }
            if (key_i)  // if we find a corresponding state i
            {
              if (find_gyro) {
                pose_key_i  = X(key_i);
                vel_key_i   = V(key_i);
                bias_key_i  = B(key_i);
                cbd_key_i   = C(key_i);
                ddAmb_key_i = N(key_i);
                omega_key_i = W(key_i);
                pose_key_j  = X(key_j);
                vel_key_j   = V(key_j);
                bias_key_j  = B(key_j);
                cbd_key_j   = C(key_j);
                ddAmb_key_j = N(key_j);
                omega_key_j = W(key_j);
                const double tau = corrected_time_gnss_meas - key_i_time;
                const double delta_t = key_j_time - key_i_time;

                std::shared_ptr<fgo::models::GPInterpolator> interpolatori;
                if (graphParams_.gpType == "WNOJ") {
                  interpolatori = std::make_shared<fgo::models::GPInterpolatorPose3WNOJ>(
                      gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6), 0, 0,
                      graphParams_.AutoDiffGPMotionPriorFactor, graphParams_.GPInterpolatedFactorCalcJacobian);
                } else if (graphParams_.gpType == "WNOA") {
                  interpolatori = std::make_shared<fgo::models::GPInterpolatorPose3WNOA>(
                      gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6), 0, 0,
                      graphParams_.AutoDiffGPMotionPriorFactor, graphParams_.GPInterpolatedFactorCalcJacobian);
                } else {
                  RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "NO gpType chosen. Please choose.");
                  return false;
                }
                //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE
                if (graphParams_.gpType == "WNOJ") {
                  interpolatori->recalculate(delta_t, tau);
                } else {
                  interpolatori->recalculate(delta_t, tau);
                }

                //PSEUDORANGE DOPPLER GP
                if (graphParams_.usePseudoRangeDoppler) {
                  RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "GPPRDR1");
                  addGPInterpolatedGNSSPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j,
                                                  omega_key_j, bias_key_i, cbd_key_i, this_gyro, meas_gnss.measMainAnt.obs,
                                                  interpolatori, 1);
                }
                if (graphParams_.usePseudoRangeDoppler && graphParams_.useDualAntenna && meas_gnss.hasDualAntenna) {
                  RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "GPPRDR2");
                  addGPInterpolatedGNSSPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j,
                                                  omega_key_j, bias_key_i, cbd_key_i, this_gyro, meas_gnss.measAuxAnt.obs,
                                                  interpolatori, 2);
                }
                //DOUBLE DIFFERENCE PSEUDORANGE GP
                if (graphParams_.useDDPseudoRange && graphParams_.useRTCMDD && meas_gnss.hasRTK) {
                  RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "GPDDPRDR1");
                  addGPInterpolatedDDPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                                this_gyro, meas_gnss.measRTCMDD.obs, meas_gnss.measRTCMDD.refSatGPS,
                                                meas_gnss.measRTCMDD.basePosRTCM, interpolatori, 1);
                }
                if (graphParams_.usePseudoRangeDoppler && graphParams_.useDualAntenna && meas_gnss.hasDualAntennaDD) {
                  RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "GPDDPRDR2");
                  addGPInterpolatedDDPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                                this_gyro, meas_gnss.measDualAntennaDD.obs,
                                                meas_gnss.measDualAntennaDD.refSatGPS, meas_gnss.measRTCMDD.basePosRTCM,
                                                interpolatori, 2);
                }
              } else
                RCLCPP_DEBUG(rclcpp::get_logger(rosLoggerName_),
                             "[EarlyObs.] GNSS Obs. in front of IMU measurements, but NO gyro found!");
            }
          } else
            RCLCPP_DEBUG(rclcpp::get_logger(rosLoggerName_),
                         "[EarlyGNSSObs.] GNSS Obs. in front of IMU measurements, but NOT interpolating!");
          continue; // Anyway, we don't need to do the followings
        }

        double sum_imu_dt = 0;  // we use this to calculated the duration between states based on dt of imu measurements
        fgo::data_types::IMUMeasurement imu_tmp = dataIMU.back();  // we need a temporary data holder of the synchronized IMU measurement
        // on the time of one
        uint counter = 0;
        gtsam::Vector3 meanAcca, meanAccg = gtsam::Vector3::Zero();

        static gtsam::Vector3 lastMeanAcca = gtsam::Vector3::Zero();
        static gtsam::Vector3 lastMeanAccg = gtsam::Vector3::Zero();

        while (imu_meas_iter != dataIMU.end())
        {
          //RCLCPP_INFO_STREAM(this->get_logger(), "Try Preintegrate IMU:" << std::fixed << imu_meas_iter->timestamp.seconds() << " vs GNSS: " << std::fixed << corrected_time_gnss_meas );
          // we want to integrate the imu measurements for this GNSS measurement in two cases
          // 1. for all IMU measurements earlier than this GNSS meas OR
          // 2. for the IMU measurement which came later than this GNSS meas, but they are still synchronized
          if (imu_meas_iter->timestamp.seconds() < corrected_time_gnss_meas ||
              imu_meas_iter->timestamp.seconds() - corrected_time_gnss_meas < graphParams_.IMUGNSSSyncTimeThreshold)
          {
            // we first integrate the measurement
            //RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "Preintegrate");

            imuPreIntegrationOPT_->integrateMeasurement(imu_meas_iter->acc,
                                                        imu_meas_iter->gyro,
                                                        imu_meas_iter->dt);

            sum_imu_dt += imu_meas_iter->dt;
            counter++;
            if (graphParams_.gpType == "WNOJ") {
              meanAcca += imu_meas_iter->acc;
              meanAccg += imu_meas_iter->gyroAcc;
            }
            timeGyroMap.push_back(std::pair<double, gtsam::Vector3>(imu_meas_iter->timestamp.seconds(), imu_meas_iter->gyro));
            // then we could delete the measurement from dataIMU
            imu_meas_iter = dataIMU.erase(imu_meas_iter);
          } else {// if the IMU measurements are later than this GNSS measurement, we stop integrating and save current imu iterator for next GNSS measurement
            imu_tmp = *imu_meas_iter;
            break;
          }
        }
        if (graphParams_.gpType == "WNOJ" && counter != 0) {
          meanAcca = current_pred_state.imuBias.correctAccelerometer(meanAcca / counter);
          meanAccg = meanAccg / counter;
          //meanAcca += current_pred_state.state.R().transpose() *
          //    fgo::utils::nedRe_Matrix(current_pred_state.state.t()).transpose() * gtsam::Vector3(0,0,fgo::constants::gravity); //TODO this correction can be done better
        }
        // now we have integrated all IMU measurements and are ready to extend the State variable
        // Attention: IN THIS MODE, THE GNSS Measurements are ALL synchronized with states so that there is NO GP interpolated GNSS factor necessary
        nStates_++;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "New State: " << nStates_ << " at dt: " << sum_imu_dt << " with Num of IMUs: " << counter);
        nDDIntAmb_  = 0;
        pose_key_j  = X(nStates_),
        vel_key_j   = V(nStates_),
        bias_key_j  = B(nStates_),
        cbd_key_j   = C(nStates_),
        ddAmb_key_j = N(nStates_),
        omega_key_j = W(nStates_),
        pose_key_i  = X(nStates_ - 1),
        vel_key_i   = V(nStates_ - 1),
        bias_key_i  = B(nStates_ - 1),
        cbd_key_i   = C(nStates_ - 1),
        ddAmb_key_i = N(nStates_ - 1),
        omega_key_i = W(nStates_ - 1);
        varIDTimestampMap.push_back(std::pair<uint,double>(nStates_, meas_gnss.measMainAnt.tow));

        //setup values
        values_.insert(pose_key_j, current_pred_state.state.pose());
        values_.insert(vel_key_j, current_pred_state.state.velocity());
        values_.insert(bias_key_j, current_pred_state.imuBias);
        values_.insert(cbd_key_j, current_pred_state.cbd);

        keyTimestampMap_[pose_key_j] =
        keyTimestampMap_[vel_key_j] =
        keyTimestampMap_[bias_key_j] =
        keyTimestampMap_[cbd_key_j] = corrected_time_gnss_meas;

        // IMU Factor
        //RCLCPP_INFO(this->get_logger(), "IMUFactor");
        boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor = //make new IMU Factor
            boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                         pose_key_j, vel_key_j,
                                                         bias_key_i, bias_key_j,
                                                         *imuPreIntegrationOPT_);
        this->push_back(imu_factor);
        imuPreIntegrationOPT_->resetIntegration();

        // Const GNSS Clock error factor
        if (graphParams_.useConstDriftFactor) {
          //RCLCPP_INFO(this->get_logger(), "ConstDriftFactor");
          this->addConstDriftFactor(cbd_key_i, cbd_key_j, sum_imu_dt);
        }

        //GP prior
        if (graphParams_.useGPPriorFactor) {
          //RCLCPP_INFO_STREAM(this->get_logger(), "meanAcca:" << std::fixed << meanAcca << "meanAccg:" << meanAccg);
          //RCLCPP_INFO_STREAM(this->get_logger(), "meanAcca:" << std::fixed << currentPredStateTmp.imuBias.correctAccelerometer(meanAcca) <<
          //"meanAccg:" <<  currentPredStateTmp.imuBias.correctGyroscope(meanAccg));
          keyTimestampMap_[omega_key_j] = corrected_time_gnss_meas;
          values_.insert(omega_key_j, current_pred_state.omega);
          //RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "GPPriorFactor" << graphParams_.gpType);
          this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, sum_imu_dt,
                                 lastMeanAcca, lastMeanAccg,
                                 meanAcca, meanAccg);
          this->addAngularFactor(omega_key_j, bias_key_j, imu_tmp);
        }

        lastMeanAcca = meanAcca;
        lastMeanAccg = meanAccg;
        meanAccg.setZero();
        meanAcca.setZero();


        //RCLCPP_INFO(this->get_logger(), "AngularRateFactor");
        //RCLCPP_INFO_STREAM(this->get_logger(), "Guess:" << std::fixed << currentPredStateTmp.omega);
        //RCLCPP_INFO_STREAM(this->get_logger(), "OmegaReal:" << std::fixed << imu_tmp.gyro);

        // GNSS Factors
        //PSEUDORANGE DOPPLER SYNCED
        if (graphParams_.usePseudoRangeDoppler) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "PRDR1 n: " << meas_gnss.measMainAnt.obs.size());
          //std::cout << "main tow: " << meas_gnss.measMainAnt.tow << std::endl;
          this->addGNSSPrDrFactor(pose_key_j, vel_key_j, bias_key_j, cbd_key_j, meas_gnss.measMainAnt.obs, imu_tmp.gyro, 1);
        }

        if (graphParams_.usePseudoRangeDoppler && graphParams_.useDualAntenna && meas_gnss.hasDualAntenna) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "PRDR2 n: " << meas_gnss.measAuxAnt.obs.size());
         // std::cout << "Aux tow: " << meas_gnss.measAuxAnt.tow << std::endl;
          this->addGNSSPrDrFactor(pose_key_j, vel_key_j, bias_key_j, cbd_key_j, meas_gnss.measAuxAnt.obs, imu_tmp.gyro, 2);
        }

        //DOUBLE DIFFERENCE PSEUDORANGE SYNCED
        if (graphParams_.useDDPseudoRange && graphParams_.useRTCMDD && meas_gnss.hasRTCMDD) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "DDPRDR1 n: " << meas_gnss.measRTCMDD.obs.size());
          this->addGNSSDDPrDrFactor(pose_key_j, vel_key_j, bias_key_j, meas_gnss.measRTCMDD.obs,
                                    meas_gnss.measRTCMDD.refSatGPS, meas_gnss.measRTCMDD.basePosRTCM, imu_tmp.gyro, 1);
        }

        if (graphParams_.useDDPseudoRange && graphParams_.useDualAntennaDD && meas_gnss.hasDualAntennaDD) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "DDPRDR2 n: " << meas_gnss.measDualAntennaDD.obs.size());
          this->addGNSSDDPrDrFactor(pose_key_j, vel_key_j, bias_key_j, meas_gnss.measDualAntennaDD.obs,
                                    meas_gnss.measDualAntennaDD.refSatGPS, meas_gnss.measRTCMDD.basePosRTCM, imu_tmp.gyro, 2);
        }

        //DOUBLE DIFFERENCE CARRIERPHASE SYNCED
        if (graphParams_.useDDCarrierPhase && graphParams_.useRTCMDD && meas_gnss.hasRTCMDD) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "DDCP1 n: " << meas_gnss.measRTCMDD.obs.size());
          this->addGNSSDDCPFactor(pose_key_j, ddAmb_key_j, meas_gnss.measRTCMDD.obs,
                                  meas_gnss.measRTCMDD.refSatGPS, meas_gnss.measRTCMDD.basePosRTCM, 1, current_pred_state);
          keyTimestampMap_[ddAmb_key_j] = corrected_time_gnss_meas;
          values_.insert(ddAmb_key_j, current_pred_state.ddIntAmb);
        }

        if (graphParams_.useDDCarrierPhase && graphParams_.useDualAntennaDD && meas_gnss.hasDualAntennaDD) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "DDCP2 n: " << meas_gnss.measDualAntennaDD.obs.size());
          this->addGNSSDDCPFactor(pose_key_j, ddAmb_key_j, meas_gnss.measDualAntennaDD.obs,
                                  meas_gnss.measDualAntennaDD.refSatGPS, meas_gnss.measRTCMDD.basePosRTCM, 2, current_pred_state);
        }

        //TIME DIFFERENCE CARRIERPHASE SYNCED  addGPInterpolatedTDNormalCPFactor
        if (graphParams_.useTDCarrierPhase) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "TDCP1 n: TBD");
         // this->addGPInterpolatedTDNormalCPFactor(pose_key_i, pose_key_j, ddAmb_key_i, ddAmb_key_j, meas_gnss.measRTCMDD.obs,
         //                         meas_gnss.measRTCMDD.refSatGPS.refSatPos,
         //                         meas_gnss.measRTCMDD.refSatGPS.refSatSVID, meas_gnss.measRTCMDD.basePosRTCM, 1, nStates_);
        }

        if (graphParams_.useTDCarrierPhase && graphParams_.useDualAntennaDD) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "TDCP2 n: TBD");
          this->addGNSSTDCPFactor(pose_key_i, pose_key_j, ddAmb_key_i, ddAmb_key_j, meas_gnss.measDualAntennaDD.obs,
                                  meas_gnss.measDualAntennaDD.refSatGPS.refSatPos,
                                  meas_gnss.measDualAntennaDD.refSatGPS.refSatSVID,
                                  meas_gnss.measRTCMDD.basePosRTCM, 2, nStates_);
        }

        sum_imu_dt = 0.; // DO NOT DELETE! MUST BE IN FRONT OF THE END OF THIS LOOP.
      }
      //set IMURest
      dataIMURest_ = dataIMU;

      //if (nStates_ == 1){
      //  ofstream os2("Matrix.txt");
      //  os2 << "A" << std::endl;
      //  os2 << this->linearize(values_)->jacobian().first << std::endl;
      //  os2 << "b" << std::endl;
      //  os2 << this->linearize(values_)->jacobian().second << std::endl;
      //}
      if(graphParams_.verbose)
      {
        values_.print();
       // RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), this->linearize(values_)->sparseJacobian_());
        RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "constructFactorGraphOnGNSS completed");
      }
      return true;
    }

    double GraphGNSSINSIntegration::optimize(data_types::State &new_state) {

      // when we call this function, it means that the optimization was triggered, no matter where the trigger came from
      // this function is then stand alone and independent of threading organization
      std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

      solver_->update(*this, values_, keyTimestampMap_);
      gtsam::Values result = solver_->calculateEstimate();


      //if(nStates_ == 420) {
      //  std::ofstream os2("/home/haoming/420solver.dot");
      //  solver_->getFactors().dot(os2, result);
      //}

     // if(nStates_ == 421) {
      //  std::ofstream os2("/home/haoming/421solver.dot");
      //  solver_->getFactors().dot(os2, result);
     // }

      gtsam::Marginals marginals = solver_->getMarginals(result); //TODO change for ISAM
      double timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
      //if (timeOpt < 0.01){
      //  rclcpp::sleep_for(std::chrono::nanoseconds(int(1e7 - timeOpt * fgo::utils::sec2nanosec)));
      //}

      if(graphParams_.verbose){
        RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "Finished graph optimization, fetching results ..., TimeOpt: " << timeOpt);
      }

      if(nStates_ == 1)
      {
        optPoseMap_.insert(std::make_pair(0, result.at<gtsam::Pose3>(X(0))));
        optVelMap_.insert(std::make_pair(0, result.at<gtsam::Vector3>(V(0))));
        if(graphParams_.useGPPriorFactor || graphParams_.UseGPInterpolatedGNSSFactor)
          optOmegaMap_.insert(std::make_pair(0, result.at<gtsam::Vector3>(W(0))));
      }

      new_state.timestamp = rclcpp::Time(keyTimestampMap_[X(nStates_)] * fgo::constants::sec2nanosec);
      new_state.state = gtsam::NavState(result.at<gtsam::Pose3>(X(nStates_)),
                                                  result.at<gtsam::Vector3>(V(nStates_)));


      optPoseMap_.insert(std::make_pair(nStates_, result.at<gtsam::Pose3>(X(nStates_))));
      optVelMap_.insert(std::make_pair(nStates_, result.at<gtsam::Vector3>(V(nStates_))));
      //RCLCPP_INFO_STREAM(this->get_logger(), "State: " << std::fixed << lastOptimizedState_.state);
      new_state.poseVar = marginals.marginalCovariance(X(nStates_));
      //std::cout << "PosVar" << lastOptimizedState_.poseVar << std::endl;
      new_state.velVar = marginals.marginalCovariance(V(nStates_));
      new_state.imuBias = result.at<gtsam::imuBias::ConstantBias>(B(nStates_));
      //RCLCPP_INFO_STREAM(this->get_logger(), "Bias: " << lastOptimizedState_.imuBias);
      new_state.imuBiasVar = marginals.marginalCovariance(B(nStates_));
      new_state.cbd = result.at<gtsam::Vector2>(C(nStates_));
      //RCLCPP_INFO_STREAM(this->get_logger(), "CBD: " << lastOptimizedState_.cbd);
      new_state.cbdVar = marginals.marginalCovariance(C(nStates_));


      if(graphParams_.useGPPriorFactor || graphParams_.UseGPInterpolatedGNSSFactor){
        new_state.omega = result.at<gtsam::Vector3>(W(nStates_));
        optOmegaMap_.insert(std::make_pair(nStates_, result.at<gtsam::Vector3>(W(nStates_))));
        //RCLCPP_INFO_STREAM(this->get_logger(), "Omega: " << lastOptimizedState_.omega);
        new_state.omegaVar = marginals.marginalCovariance(W(nStates_));
      }

      if(graphParams_.useLIOSAM && !LIOSAMResults_.empty())
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(rosLoggerName_), "UPDATING LIOSAM OPTIMIZED POSE ...");

        static std::shared_ptr<fgo::models::GPInterpolator> interpolatori, interpolatorj;
        if (graphParams_.gpType == "WNOJ"){
          interpolatori = std::make_shared<fgo::models::GPInterpolatorPose3WNOJ>(
              gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6));
          interpolatorj = std::make_shared<fgo::models::GPInterpolatorPose3WNOJ>(
              gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6));
        } else if (graphParams_.gpType == "WNOA") {
          interpolatori = std::make_shared<fgo::models::GPInterpolatorPose3WNOA>(
              gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6));
          interpolatorj = std::make_shared<fgo::models::GPInterpolatorPose3WNOA>(
              gtsam::noiseModel::Gaussian::Covariance(graphParams_.QcGPInterpolator * gtsam::I_6x6));
        } else {
          RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "NO gpType chosen. Please choose.");
          return false;
        }

        auto currentKeyIndexTimestampMap = solver_->keyIndexTimestamps();
        for(auto& odom : LIOSAMResults_)
        {
          //continue;
          gtsam::Pose3 poseI, poseJ;
          if(odom.keyJSynchronized && odom.keyISynchronized)
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: odom.keyISynchronized && odom.keyJSynchronized");
            if(result.exists(X(odom.keyIndexII)))
            {
              poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 1: Continue because no keyindexII in results, should be marginalized out!");
              poseI = odom.posePreviousIMUECEFQueried;
            }
            if(result.exists(X(odom.keyIndexJI)))
            {
              poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 1: Continue because no keyindexJI in results, should be marginalized out!");
              poseJ = odom.poseCurrentIMUECEFQueried;
            }

          }
          else if(odom.keyISynchronized && !odom.keyJSynchronized)
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: odom.keyISynchronized && !odom.keyJSynchronized");
            if(result.exists(X(odom.keyIndexII)))
            {
              poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 2: Continue because no keyindexII in results, should be marginalized out!");
              poseI = odom.posePreviousIMUECEFQueried;
            }
            interpolatorj->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI, odom.accJI.head(3), odom.accJI.tail(3), odom.accJJ.head(3), odom.accJJ.tail(3));

            if(result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ)) )
            {
              poseJ = interpolatorj->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)), result.at<gtsam::Vector3>(V(odom.keyIndexJI)), result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                     result.at<gtsam::Pose3>(X(odom.keyIndexJJ)), result.at<gtsam::Vector3>(V(odom.keyIndexJJ)), result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 2: Continue because no keyIndexJI or keyIndexJI in results, should be marginalized out!");
              poseJ = odom.poseCurrentIMUECEFQueried;
            }
          }
          else if(!odom.keyISynchronized && odom.keyJSynchronized)
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: !odom.keyISynchronized && odom.keyJSynchronized");
            if(result.exists(X(odom.keyIndexJI)))
            {
              poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: Continue because no keyindexJI in results, should be marginalized out!");
              poseJ = odom.poseCurrentIMUECEFQueried;
            }

            interpolatori->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII, odom.accII.head(3), odom.accII.tail(3), odom.accIJ.head(3), odom.accIJ.tail(3));

            if(result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ)))
            {
              poseI = interpolatori->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)), result.at<gtsam::Vector3>(V(odom.keyIndexII)), result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                     result.at<gtsam::Pose3>(X(odom.keyIndexIJ)), result.at<gtsam::Vector3>(V(odom.keyIndexIJ)), result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
              poseI = odom.posePreviousIMUECEFQueried;
            }

          }
          else if(!odom.keyISynchronized && !odom.keyJSynchronized)
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 3: !odom.keyISynchronized && !odom.keyJSynchronized");
            interpolatori->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII, odom.accII.head(3), odom.accII.tail(3), odom.accIJ.head(3), odom.accIJ.tail(3));
            interpolatorj->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI, odom.accJI.head(3), odom.accJI.tail(3), odom.accJJ.head(3), odom.accJJ.tail(3));

            if(result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ)))
            {
              poseI = interpolatori->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)), result.at<gtsam::Vector3>(V(odom.keyIndexII)), result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                     result.at<gtsam::Pose3>(X(odom.keyIndexIJ)), result.at<gtsam::Vector3>(V(odom.keyIndexIJ)), result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 4: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
              poseI = odom.posePreviousIMUECEFQueried;
            }

            if(result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ)))
            {
              poseJ = interpolatorj->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)), result.at<gtsam::Vector3>(V(odom.keyIndexJI)), result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                     result.at<gtsam::Pose3>(X(odom.keyIndexJJ)), result.at<gtsam::Vector3>(V(odom.keyIndexJJ)), result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));
            }
            else
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Case 4: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
              poseJ = odom.poseCurrentIMUECEFQueried;
            }

          }
          else
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Can't update key cloud pose for odom with scan index " << odom.frameIndexPrevious << " and " << odom.frameIndexCurrent);
            continue;
          }

          //RCLCPP_ERROR(rclcpp::get_logger(rosLoggerName_), "----------------- LIOSAM RESULT -----------------");

          //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose I: \n" << std::fixed <<poseI);
          //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose J: \n" << std::fixed<< poseJ);

          //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose Between: \n" << std::fixed << poseI.between(poseJ));

          //RCLCPP_ERROR(rclcpp::get_logger(rosLoggerName_), "----------------- LIOSAM RESULT DONE -----------------");


          LIOSAM_->updateCloudKeyPose(odom.frameIndexPrevious, poseI, odom.frameIndexCurrent, poseJ);
        }
        LIOSAMResults_.clear();
        //RCLCPP_ERROR_STREAM(rclcpp::get_logger(rosLoggerName_), "UPDATING LIOSAM OPTIMIZED POSE DONE!");
      }

      if(graphParams_.useDDCarrierPhase)
      {
        //get Cov for Lambda (superfast)
        gtsam::KeyVector keyList;
        keyList.push_back(X(nStates_));
        if (result.exists(N(nStates_))) {
          auto intAmb = result.at<gtsam::Vector>(N(nStates_));
          for (int i = 0; i < intAmb.size(); i++) {
            if (!IntAmbFixed_[i])
              new_state.ddIntAmb[i] = intAmb(i);
          }
          //RCLCPP_INFO_STREAM(get_logger(), "MixedIntAmb: " << lastOptimizedState_.ddIntAmb.transpose());
          new_state.ddIntAmbVar = marginals.marginalCovariance(N(nStates_));
          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "ddIntCovariance: " << std::endl << new_state.ddIntAmbVar);
          keyList.push_back(N(nStates_));
        } else if (result.exists(N(nStates_ - 1)) && graphParams_.optStrategy == fgo::params::OptimizationStrategy::ONIMU && nStates_ >= 1) {
          RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "1.7");
          new_state.ddIntAmb = result.at<gtsam::Vector>(N(nStates_ - 1));
          new_state.ddIntAmbVar = marginals.marginalCovariance(N(nStates_ - 1));
          keyList.push_back(N(nStates_ - 1));
        }
        //std::cout << std::fixed << "AmbInt:" << lastOptimizedState_.ddIntAmb.transpose() << std::endl;
        //Lambda Algorithm takes ca 0.0001 sec, so this is no time problem
        //std::cout << std::fixed << "position before fixed Int:" << lastOptimizedState_.state.position().transpose() << std::endl;
        //std::cout << std::fixed << "FloatInt:" << lastOptimizedState_.ddIntAmb.transpose() << std::endl;
        bool hasNFixed = false;
        for (const auto &x: IntAmbFixed_) {
          if (x) {
            hasNFixed = true;
            break;
          }
        }
        if (graphParams_.lambdaAlgorithm && !hasNFixed && keyList.size() == 2)
        { // && lastOptimizedState_.ddIntAmbVar.determinant() < 0.1
          RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "Start Lambda");
          gtsam::Matrix marginalCovariance = marginals.jointMarginalCovariance(
              keyList).fullMatrix().diagonal().asDiagonal();
          //gtsam::Matrix marginalCovariance = ;
          gtsam::Vector fixedInteger = new_state.ddIntAmb;
          gtsam::Vector3 fixedPosition(0, 0, 0);
          if (fgo::utils::GNSS::solveIntegerAmbiguity(marginalCovariance,
                                                      fixedInteger,
                                                      fixedPosition,
                                                      new_state,
                                                      rosLoggerName_)) {
            //lastOptimizedState_.state = gtsam::NavState(lastOptimizedState_.state.attitude(), fixedPosition,
            //                                            lastOptimizedState_.state.v()); //This correction might be weird
            new_state.ddIntAmb = fixedInteger;
            std::cout << std::fixed << "FixedInt:" << new_state.ddIntAmb.transpose() << std::endl;
            for (auto &&i : IntAmbFixed_)
              i = true;
          }
        }
      }

      currentKeyIndexTimestampMap_ = solver_->keyIndexTimestamps();

      if(nStates_ == 0)
      {
        fgo::data_types::State tmp;
        tmp.state = gtsam::NavState(result.at<gtsam::Pose3>(X(0)),
                                    result.at<gtsam::Vector3>(V(0)));
        tmp.timestamp =  rclcpp::Time(keyTimestampMap_[X(0)] * fgo::constants::sec2nanosec);

        if(graphParams_.useGPPriorFactor || graphParams_.UseGPInterpolatedGNSSFactor) {
          tmp.omega = result.at<gtsam::Vector3>(W(0));
        }

        tmp.imuBias = result.at<gtsam::imuBias::ConstantBias>(B(0));
        tmp.cbd = result.at<gtsam::Vector2>(C(0));
        fgoOptStateBuffer_.update_buffer(tmp, tmp.timestamp);
      }
      fgoOptStateBuffer_.update_buffer(new_state, new_state.timestamp);

      if(graphParams_.useLIOSAM)
      {
        for(const auto& keyIndexTs : currentKeyIndexTimestampMap_)
        {
          fgo::data_types::QueryStateInput input;
          input.pose = result.at<gtsam::Pose3>(X(keyIndexTs.first));
          input.vel = result.at<gtsam::Vector3>(V(keyIndexTs.first));
          if(graphParams_.useGPPriorFactor || graphParams_.UseGPInterpolatedGNSSFactor) {
            input.omega = result.at<gtsam::Vector3>(W(keyIndexTs.first));
          }
          LIOSAM_->updateOptPose(keyIndexTs.first, rclcpp::Time(keyIndexTs.second * fgo::constants::sec2nanosec), input);
        }
        LIOSAM_->updateKeyIndexTimestampMap(currentKeyIndexTimestampMap_);
      }

     // std::cout << "------------------------------------currentKeyIndexTimestampMap_: ---------------------------" << std::endl;
     // for(const auto& p : currentKeyIndexTimestampMap_)
      //  std::cout << std::fixed << "Key: " << p.first << " Time: " << p.second << std::endl;
      //values_.clear(); keyTimestampMap_.clear();
      this->resetGraph();
      return timeOpt;
    }
}
