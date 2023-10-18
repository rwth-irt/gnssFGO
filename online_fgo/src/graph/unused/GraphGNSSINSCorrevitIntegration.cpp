//
// Created by haoming on 22.06.22.
//

#include "graphs/GraphGNSSINSCorrevitIntegration.h"

namespace fgo::graphs
{

    bool GraphGNSSINSCorrevit::constructFactorGraphOnIMU(std::vector<fgo::data_types::IMUMeasurement>& dataIMU,
                                                         std::vector<fgo::data_types::GNSSMeasurement>& dataGNSS,
                                                         std::vector<fgo::data_types::CorrevitObs>& dataCorrevit,
                                                         std::vector<fgo::data_types::DumperOdom>& dataDumperOdom,
                                                         fgo::data_types::State& currentPredState) {
      RCLCPP_INFO(rclcpp::get_logger(rosLoggerName_), "GraphGNSSINSCorrevit: Constructing factor graph on IMU ...");
      // example: imu on 100Hz and we optimize on 10 Hz, then we should count 10 imu measurements
      static uint notifyCounter = graphParams_.IMUMeasurementFrequency / graphParams_.optFrequency;

      static boost::circular_buffer<std::pair<size_t, double>> varIDTimestampMap(20 * graphParams_.smootherLag * graphParams_.optFrequency);
      static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(20 * graphParams_.IMUMeasurementFrequency / graphParams_.optFrequency);

      static gtsam::Key lastStateJ = -1;
      //static double corrected_time_last_gnss = -1.0;
      //we buffer lastGNSSmeas, such that we can create a TDCP factor between first new GNSS meas and last old GNSS
      static fgo::data_types::GNSSMeasurement lastGNSSMeas;
      //we buffer a pointer for the interpolator of the next GNSS measuremnt
      static std::shared_ptr<fgo::models::GPInterpolator> interpolatorj;
      //for WNOJ we need the 2nd derivation of the position acc and gyroAcc for the states, therefore we create a map where we buffer
      static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> accAandaccGMap(20 * graphParams_.smootherLag * graphParams_.optFrequency);
      //static struct for DDCP to store not slipped satellites
      static std::list<std::pair<uint32_t, bool>> notSlippedSatellites;

      static boost::circular_buffer<fgo::data_types::GNSSMeasurement> restGNSSMesa(10);

      //get all GNSS Measurements

      if (dataIMU.empty() || !isStateInited_) {
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "constructFactorGraphOnIMU: No IMU measurement, waiting ..."); // this shouldn't happen
        return false;
      }

      if(dataGNSS.empty())
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "constructFactorGraphOnIMU: No GNSS measurement!"); // this shouldn't happen

      if(dataCorrevit.empty())
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "constructFactorGraphOnIMU: No Correvit measurement!"); // this shouldn't happen

      if(dataDumperOdom.empty())
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "constructFactorGraphOnIMU: No Odom measurement!"); // this shouldn't happen

      RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_),
                         "----------- Start constructFactorGraphOnIMU ----------- \n" <<
                         "Size IMU Meas.: " << dataIMU.size() << "\n" <<
                         "Size GNSS Meas.: " << dataGNSS.size() << "\n" <<
                         "Size Correvit Meas.: " << dataCorrevit.size() << "\n" <<
                         "Size Odom Meas.: " << dataDumperOdom.size() << "\n" <<
                         ""
                         ); //  << dataIMURest_.size());
      // we copy the data of last optimized state for safe, because this is used in imu cb.

      std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
          std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                     currentPredState.imuBias);

      // Step 2 : init values
      //declare const variables for all factor;
      gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, ddAmb_key_j, omega_key_j,
                 pose_key_i, vel_key_i, bias_key_i, cbd_key_i, ddAmb_key_i, omega_key_i;

      const auto first_imu_meas_timestamp = dataIMU.front().timestamp.seconds();  // in double
      const auto last_imu_meas_timestamp = dataIMU.back().timestamp.seconds();  // in double

      // Step 1: extend state variables.
      // Because we assume, that the IMU measurements are coming on time with only minimal delay,
      // (this assumption should be okay, since we are counting the IMU down, which is like to get the mean transfer frequency)
      // we count the IMU measurement after its defined frequency and wanted optimization frequency to create state variable
      // e.g. IMU raw measurements on 100Hz, we want to optimize in 10Hz, then we pre-integrate 10 IMU measurement and make a state
      // PS: the size of dataIMU MUST be exactly = n x States x opt_frequency, e.g. 100 IMU measurements for 10 states at 10hz opt

      double sum_imu_dt = 0.;
      uint counter_imu = 0;
      uint64_t num_state_started = nStates_;
      gtsam::Vector3 meanAcca, meanAccg = gtsam::Vector3();
      size_t i = 0;

      for(; i < dataIMU.size(); i++)
      {
        auto current_imu_meas = dataIMU[i];
        auto current_imu_meas_timestamp = current_imu_meas.timestamp.seconds();
        sum_imu_dt += current_imu_meas.dt;
        counter_imu ++;
        timeGyroMap.push_back(std::make_pair(current_imu_meas_timestamp, current_imu_meas.gyro));
        imuPreIntegrationOPT_->integrateMeasurement(current_imu_meas.acc,
                                                    current_imu_meas.gyro,
                                                    current_imu_meas.dt);


        if(graphParams_.gpType == "WNOJ")
        {
          meanAcca += current_imu_meas.acc;
          meanAccg += current_imu_meas.gyroAcc;
        }

        if( ( (i + 1) % notifyCounter) == 0 )
        {
          nStates_++;
          pose_key_j  = X(nStates_);
          vel_key_j   = V(nStates_);
          bias_key_j  = B(nStates_);
          cbd_key_j   = C(nStates_);
          omega_key_j = W(nStates_);
          pose_key_i  = X(nStates_ - 1);
          vel_key_i   = V(nStates_ - 1);
          bias_key_i  = B(nStates_ - 1);
          cbd_key_i   = C(nStates_ - 1);
          omega_key_i = W(nStates_ - 1);

          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "FGConIMU: creating state variable " << nStates_ << " at: " << std::fixed <<current_imu_meas.timestamp.seconds());

          varIDTimestampMap.push_back(std::make_pair(nStates_, current_imu_meas_timestamp));

          values_.insert(pose_key_i, currentPredState.state.pose());
          values_.insert(vel_key_j, currentPredState.state.velocity());
          values_.insert(bias_key_j, currentPredState.imuBias);
          values_.insert(cbd_key_j, currentPredState.cbd);

          keyTimestampMap_[pose_key_j]  =
          keyTimestampMap_[vel_key_j]   =
          keyTimestampMap_[bias_key_j]  =
          keyTimestampMap_[cbd_key_j]   = current_imu_meas_timestamp;

          if (graphParams_.useDDCarrierPhase)
          {
            // ToDo 23.03: @Lars do we need to put ddAmb into values? => Value must be added later after created factor
            ddAmb_key_i = N(nStates_-1);
            ddAmb_key_j = N(nStates_);
            keyTimestampMap_[ddAmb_key_j] = current_imu_meas.timestamp.seconds();
          }

          //add acc to map
          if (graphParams_.gpType == "WNOJ") {
            meanAcca = currentPredState.imuBias.correctAccelerometer(meanAcca / counter_imu);
            meanAccg = currentPredState.imuBias.correctGyroscope(meanAccg / counter_imu);
            accAandaccGMap.push_back(std::pair<uint,gtsam::Vector6>(
                nStates_, (gtsam::Vector6() << meanAcca, meanAccg).finished()));
          }

          // IMU Factor
          boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor = //make new IMU Factor
              boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                           pose_key_j, vel_key_j,
                                                           bias_key_i, bias_key_j,
                                                           *imuPreIntegrationOPT_);
          this->push_back(imu_factor);

          // Const GNSS Clock error factor
          if (graphParams_.useConstDriftFactor) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "FGConIMU: state variable " << nStates_ << " with cbd factor.");
            addConstDriftFactor(cbd_key_i, cbd_key_j, sum_imu_dt);
          }

          //const vel prior factor
          if (graphParams_.useMMFactor) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "FGConIMU: state variable " << nStates_ << " with MM factor.");
            addMotionModelFactor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, sum_imu_dt);
          }

          //GP prior
          if (graphParams_.useGPPriorFactor) {
            //RCLCPP_INFO_STREAM(this->get_logger(), "meanAcca:" << std::fixed << meanAcca << "meanAccg:" << meanAccg);
            //RCLCPP_INFO_STREAM(this->get_logger(), "meanAcca:" << std::fixed << currentPredStateTmp.imuBias.correctAccelerometer(meanAcca) <<
            //"meanAccg:" <<  currentPredStateTmp.imuBias.correctGyroscope(meanAccg));
            keyTimestampMap_[omega_key_j] = current_imu_meas.timestamp.seconds(); //TODO
            values_.insert(omega_key_j, currentPredState.omega);
            RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "GPPriorFactor " << graphParams_.gpType);
            this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, sum_imu_dt,
                                   meanAcca, meanAccg);
            this->addAngularFactor(omega_key_j, bias_key_j, current_imu_meas);
          }

          // we reset sim_imu_dt for next interpolation.
          sum_imu_dt = 0.;
          meanAccg.setZero();
          meanAcca.setZero();

          currentPredState.state = imuPreIntegrationOPT_->predict(currentPredState.state, currentPredState.imuBias);
          imuPreIntegrationOPT_->resetIntegration();
        }
      }

      RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_),
                         "FGConIMU: end of imu iteration: created " <<
                         (nStates_ - num_state_started) <<
                         " states started at" << num_state_started <<
                         " with " << i + 1 << " imu measurements.");

      //create GP interpolators for the factor
      std::shared_ptr<fgo::models::GPInterpolator> interpolatori;
      if (graphParams_.gpType == "WNOJ"){
        interpolatori = std::make_shared<fgo::models::GPInterpolatorPose3WNOJO>(
            gtsam::noiseModel::Gaussian::Covariance(graphParams_.Qc * gtsam::I_6x6));
      } else if (graphParams_.gpType == "WNOA") {
        interpolatori = std::make_shared<fgo::models::GaussianProcessInterpolatorPose3WNOAO>(
            gtsam::noiseModel::Gaussian::Covariance(graphParams_.Qc * gtsam::I_6x6));
      } else {
        RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "NO gpType chosen. Please choose.");
        return false;
      }

      // TODO: START GNSS
      auto gnssIter = dataGNSS.begin();
      while (gnssIter != dataGNSS.end())
      {
        // we iterate then though all gnss measurements, trying to find the corresponding one, this could be exhausted,
        // thus it must be considered carefully.

        // firstly, we check, whether there are gnss measurements, which are delayed and should be integrated in LAST optimization epochs!
        auto corrected_time_gnss_meas = gnssIter->measMainAnt.timestamp.seconds() - gnssIter->measMainAnt.delay;   // in double

        gtsam::Vector3 this_gyro;
        bool find_gyro = false;
        auto timeGyroIter = timeGyroMap.rbegin();
        while (timeGyroIter != timeGyroMap.rend()) {
          if (abs(timeGyroIter->first - corrected_time_gnss_meas) < graphParams_.IMUGNSSSyncTimeThreshold) {
            find_gyro = true;
            this_gyro = currentPredState.imuBias.correctGyroscope(timeGyroIter->second); // ToDo: to be checked.
            break;
          }
          timeGyroIter++;
        }

        if(corrected_time_gnss_meas < first_imu_meas_timestamp &&
           abs(corrected_time_gnss_meas - first_imu_meas_timestamp) > graphParams_.StateGNSSSyncTimeThreshold)
        {
          RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "!!!!!!!!!!!! GNSS Obs. in front of IMU measurements! not implemented!");
          continue;
        }
        else
        {
          bool foundI = false, isStateMeasSyncWithI = false;
          size_t idStateI = 0, idStateMin = 0, idStateJ = 0;
          double timeStateI, timeStateJ;
          double minDurationStateGNSS = std::numeric_limits<double>::max();

          this->findStateForMeasurement(varIDTimestampMap, corrected_time_gnss_meas, foundI, isStateMeasSyncWithI,
                                        idStateI, idStateMin, idStateJ,timeStateI, timeStateJ, minDurationStateGNSS,
                                        graphParams_);

          RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "Found State I: " << idStateI << " and J: " << idStateJ);

          if (!foundI){
            RCLCPP_WARN(rclcpp::get_logger(rosLoggerName_), "State I or J couldn't be found in varIDTimestampMap.");
            gnssIter++;
            continue;
          }

          pose_key_i   = X(idStateI);
          vel_key_i    = V(idStateI);
          omega_key_i  = W(idStateI);
          bias_key_i   = B(idStateI);
          cbd_key_i    = C(idStateI);
          ddAmb_key_i  = N(idStateI);

          pose_key_j   = X(idStateJ);
          vel_key_j    = V(idStateJ);
          omega_key_j  = W(idStateJ);
          bias_key_j   = B(idStateJ);
          cbd_key_j    = C(idStateJ);
          ddAmb_key_j  = N(idStateJ);

          const double tauI = corrected_time_gnss_meas - timeStateI;
          const double delta_t = timeStateJ - timeStateI;

          if (graphParams_.gpType == "WNOJ") {
            gtsam::Vector6 accI, accJ;
            auto accMapIterator = accAandaccGMap.rbegin();
            while (accMapIterator != accAandaccGMap.rend()) {
              if (accMapIterator->first == idStateI) {
                accI = accMapIterator->second;
                accMapIterator++;
                accJ = accMapIterator->second;
                break;
              }
              accMapIterator++;
            }
            interpolatori->recalculate(delta_t, tauI, accI, accJ);
          } else {
            interpolatori->recalculate(delta_t, tauI);
          }

          if (isStateMeasSyncWithI) {
            // now we found a state which is synchronized with the GNSS obs
            RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "[GNSSObs.] Found time synchronized state " << idStateMin <<
                                                                                                               " with time difference: "
                                                                                                               << minDurationStateGNSS);
            gtsam::Key pose_key, vel_key, omega_key, cbd_key, ddAmb_key;

            if (idStateMin == idStateI){
              //PSEUDORANGE DOPPLER SYNCED
              pose_key = pose_key_i;
              vel_key = vel_key_i;
              omega_key = omega_key_i;
              cbd_key = cbd_key_i;
              ddAmb_key = ddAmb_key_i;
            }
              //if (idStateMin == idStateJ)
            else{
              pose_key = pose_key_j;
              vel_key = vel_key_j;
              omega_key = omega_key_j;
              cbd_key = cbd_key_j;
              ddAmb_key = ddAmb_key_j;
            }

            //PSEUDORANGE DOPPLER SYNCED
            if (graphParams_.usePseudoRangeDoppler) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "PRDR1 n: " << gnssIter->measMainAnt.obs.size());
              this->addGNSSPrDrFactor(pose_key, vel_key, bias_key_j, cbd_key, gnssIter->measMainAnt.obs, this_gyro, 1);
            }
          }
          else if (corrected_time_gnss_meas < last_imu_meas_timestamp)
          {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "[GNSSObs.] CAN'T found synchronized between " << idStateI << " and " << idStateJ <<
                                                                                                                " with time difference: "
                                                                                                                << minDurationStateGNSS);
            //PSEUDORANGE DOPPLER GP
            if (graphParams_.usePseudoRangeDoppler) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "GPPRDR1 n: " << gnssIter->measMainAnt.obs.size());
              this->addGPInterpolatedGNSSPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j,
                                                    omega_key_j, cbd_key_i, this_gyro, gnssIter->measMainAnt.obs,
                                                    interpolatori, 1);
            }
          }
          else
          {
            // In this case, this gnss meas is behind all imu meas, then we cache it
            // NOTICES: this shouldn't happen, if so, there must be sth wrong!
            restGNSSMesa.push_back(*gnssIter);
          }
          lastStateJ = idStateJ;
          //corrected_time_last_gnss = corrected_time_gnss_meas;
          interpolatorj = interpolatori;
          gnssIter++;
        }
      }

      // TODO: START Correvit
      auto correvitIter = dataCorrevit.begin();
      while(correvitIter != dataCorrevit.end())
      {

      }







      return true;
    }




}
