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

#ifndef ONLINE_FGO_GNSSUTILS_H
#define ONLINE_FGO_GNSSUTILS_H

#pragma once

#include <bitset>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <irt_nav_msgs/msg/gnss_correction.hpp>
#include <irt_nav_msgs/msg/sat_label.hpp>
#include "data/DataTypes.h"
#include "utils/NavigationTools.h"
#include "utils/LambdaAlgorithm.h"

namespace fgo::utils::GNSS {

    inline void getRTKCorrectionsFromROSMsg(const std::vector<irt_nav_msgs::msg::GNSSCorrection>& corrections,
                                            std::vector<irt_nav_msgs::msg::SatLabel>& labels)
    {
      for(auto& label : labels)
      {
        for (const auto& corr : corrections)
        {
          if(corr.prn == label.prn)
          {
            label.psr_correction = corr.psr_correction;
            break;
          }
        }
      }
    }

    inline std::map<uint32_t, double> GetInitPRNLocktimeMap(size_t size = 40)
    {
      std::map<uint32_t, double> map;
      for(size_t i = 0; i < size; i++)
        map.insert(std::make_pair(i + 1, -1.));
      return map;
    }

    /**
     *
     * @param method
     * @param measVar
     * @param lambda
     * @param elev
     * @param snr
     * @param logger_name
     * @return
     */
    inline double calculateDDVariance(const std::string &method,
                                 const double &measVar,
                                 double const &lambda,
                                 const double &elev,
                                 const double &snr,
                                 const std::string& logger_name)
     {
       if(method == "ELEV" && elev != 0.0)
       {
         return 1/pow(sin(elev),2) * lambda * measVar;
       }
       else if (method == "SNR" && snr != 0.0 && elev != 0.0)
       {
         return fgo::utils::eles2nVariance(snr, elev) * lambda * measVar;
       } else if(method == "STD")
       {
         return lambda * measVar;
       } else
       {
         RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "The chosen VarMethod cant be used or a wrong one is chosen.");
         return lambda * measVar;
       }
     }

     inline void checkCycleSlipByLocktime(fgo::data_types::GNSSMeasurementEpoch& measEpoch,
                                          std::map<uint32_t, double>& PRNLocktimeMap,
                                          std::vector<uint32_t>& lastPRNList)
     {
       std::vector<uint32_t> currentPRNs;
       for(const auto& obs : measEpoch.obs)
       {
         currentPRNs.emplace_back(obs.satId);
       }
      if(!lastPRNList.empty())
      {
        for(const auto& prn : lastPRNList)
        {
          auto prnIter = std::find(currentPRNs.begin(), currentPRNs.end(), prn);
          if(prnIter == currentPRNs.end())
          {
            // this prn is not in the current prn list, sat lost
            PRNLocktimeMap[prn - 1] = -1.;
          }
        }
      }

      for(auto& obs : measEpoch.obs)
      {
        if(PRNLocktimeMap[obs.satId - 1] == -1.0)
        {
          obs.cycleSlip = true;
          PRNLocktimeMap[obs.satId - 1] = obs.locktime;
          continue;
        }
        if(obs.locktime == PRNLocktimeMap[obs.satId - 1] + .1)
        {
          // this obs was kept tracked
          obs.cycleSlip = false;
        }
        else
        {
          obs.cycleSlip = true;
        }
        PRNLocktimeMap[obs.satId - 1] = obs.locktime;
      }

       lastPRNList = currentPRNs;
     }
     /**
      * https://gssc.esa.int/navipedia/index.php/Examples_of_single_frequency_Cycle-Slip_Detectors
      * @param data
      * @param structRTCM
      * @param structAux
      * @param reset
      */
     inline void checkCycleSlip(fgo::data_types::GNSSMeasurementEpoch& measEpoch,
                         uint& lastRefSatID,
                         std::vector <fgo::data_types::CycleSlipStruct> &cycleSlipStruct,
                         bool reset)
                         {
       if (lastRefSatID != measEpoch.refSatGPS.refSatSVID || reset)
       {
         for (auto &&item: measEpoch.obs)
         {
           item.cycleSlip = true;
         }
         cycleSlipStruct.clear();
       }
       lastRefSatID = measEpoch.refSatGPS.refSatSVID;

       for (auto &&obs: measEpoch.obs)
       {
         bool cycleSlip = true;
         bool foundinCS = false; //double timeLastSlip = data.timestamp.seconds() - data.delay;
         auto satIDi = obs.satId;
         for (auto &cs: cycleSlipStruct)
         {
           if (satIDi == cs.satID)
           {
             foundinCS = true;
             cs.connection = true;
             double diffCpPr = obs.pr - fgo::constants::lambda_L1 * obs.cp;
             if (abs(diffCpPr - cs.md) > 1.5 * sqrt(cs.sd2) || cs.N == 0)
             {
               //Reset
               cs.N = 0;
               cs.md = 0;
               cs.md2 = 0;
               cs.sd2 = 0;
               //.timeLastSlip = timeLastSlip;
             } else
             {
               cycleSlip = false;
             }
              //timeLastSlip = cs.timeLastSlip;
              cs.N++;
              int N = cs.N;
              cs.md = 1.0 / N * (diffCpPr + (N - 1) * cs.md);
              cs.md2 = 1.0 / N * (pow(diffCpPr, 2) + (N - 1) * cs.md2);
              cs.sd2 = 1.0 / N * ((N - 1) * (cs.md2 - pow(cs.md, 2)) + 0.5); //weighted average
              break;
           }
         }
         if (!foundinCS)
         {
           fgo::data_types::CycleSlipStruct ns; //new Satellite
           ns.satID = satIDi;
           ns.N = 1;
           ns.md = fgo::constants::lambda_L1 * obs.cp - obs.pr;
           ns.md2 = pow(ns.md, 2);
           ns.sd2 = 0.5;
           ns.connection = true;
           cycleSlipStruct.push_back(ns);
         }
         obs.cycleSlip = cycleSlip;
         //obs.timeLastSlip = timeLastSlip;
       }
       for (auto &&cs: cycleSlipStruct)
       {
         if (!cs.connection && cs.N != 0)
         {
           cs.N = 0;
           cs.md = 0;
           cs.md2 = 0;
           cs.sd2 = 0;
         }
         cs.connection = false;
       }
     }

      /**
       *
       * @param marginalCovariance
       * @param fixedIntegers
       * @param fixedPosition
       * @param fgo_state
       * @param logger_name
       * @return
       */

      inline bool solveIntegerAmbiguity(const gtsam::Matrix &marginalCovariance,
                                 gtsam::Vector &fixedIntegers,
                                 gtsam::Point3 &fixedPosition,
                                 const fgo::data_types::State& fgo_state,
                                 const std::string& logger_name)
     {
        long n = fgo_state.ddIntAmb.rows();
        int m = 2; //number of sol generated
        VectorXd s = VectorXd::Zero(m);
        MatrixXd F = MatrixXd::Zero(n, m);
        VectorXd floatA = fgo_state.ddIntAmb;
        MatrixXd Q = marginalCovariance.block(0, 0, n, n);
        //Q = MatrixXd::Identity(n,n) * 1e-1;
        //std::cout << Q << std::endl;
        std::cout << "Ready for solveIntAmb" << std::endl;
        if (fgo::utils::lambda(floatA, Q, F, s, m) == 0)
        {
          if(s(0)==0.0){
            RCLCPP_INFO(rclcpp::get_logger(logger_name), "s(0) = 0, in solveIntAmb");
            return false;
          }
          RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "0: " << F.col(0).transpose());
          RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "1: " << F.col(1).transpose());

          if (s(1) / s(0) > 1.5) { //1.5
            RCLCPP_INFO(rclcpp::get_logger(logger_name), "solveIntegerAmbiguity successful");
            //stateb = statebfloat - QbaQa^-1(afloat - ainteger)
            gtsam::Point3 correctionTerm =
                marginalCovariance.block(n+3, 0, 3, n) * Q * (floatA - F.col(0));
            fixedPosition = fgo_state.state.position() - correctionTerm;
            fixedIntegers = F.col(0);
            return true;
          } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "S1/S0 ratio smaller 1.5. S1: " << s(1) << " S0: " << s(0));
            return false;
          }
        } else {
          //lambda algorithm wasnt successful
          RCLCPP_WARN(rclcpp::get_logger(logger_name), "LAMBDA-Algorithm wasn't successful.");
          return false;
        }
     }

     inline data_types::GNSSSolutionType getOEM7PVTSolutionType(uint32_t type)
     {
       switch (type) {
         case 0: {
           return data_types::GNSSSolutionType::NO_SOLUTION;
         }
         case 16: case 17: case 18:  case 51:  case 52:  case 53: {
           return data_types::GNSSSolutionType::SINGLE;
         }
         case 32: case 34:  case 55: {
           return data_types::GNSSSolutionType::RTKFLOAT;
         }
         case 48: case 49: case 50:  case 56: {
           return data_types::GNSSSolutionType::RTKFIX;
         }
         default:
           return data_types::GNSSSolutionType::SINGLE;
       }
     }

     struct UbxPVTFlag
     {
          bool gnssFixOK;
          bool diffSoln;
          bool reserved0;
          bool reserved1;
          bool psmState;
          bool headVehValid;
          bool carrSoln1;
          bool carrSoln2;
     };

    inline data_types::GNSSSolutionType getUbloxSolutionType(uint8_t fix_type, uint8_t flags)
    {
      const auto bit = std::bitset<8>(flags);
      const auto ubx_pvt_type = UbxPVTFlag{bit[0], bit[1], bit[2], bit[3], bit[4], bit[5], bit[6], bit[7]};

      if(!ubx_pvt_type.gnssFixOK)
        return data_types::GNSSSolutionType::NO_SOLUTION;

      if(!ubx_pvt_type.carrSoln1 && !ubx_pvt_type.carrSoln2)
        return data_types::GNSSSolutionType::SINGLE;
      else if(ubx_pvt_type.carrSoln1 && !ubx_pvt_type.carrSoln2)
        return data_types::GNSSSolutionType::RTKFLOAT;
      else
        return data_types::GNSSSolutionType::RTKFIX;
    }

    inline data_types::GNSSSolutionType getSBFSolutionType(uint8_t error, uint8_t type)
    {
      /*
       * switch Error
              case SOL_COMPUTED
                  SBF_Error = 0;
              case INSUFFICIENT_OBS
                  SBF_Error = 1;
              case NO_CONVERGENCE
                  SBF_Error = 5;
              case COV_TRACE
                  SBF_Error = 9;
              case Not yet converged from cold start
                  SBF_Error = 5;
              case 13 INTEGRITY_WARNING
                  SBF_Error = 9;
              case 20 UNAUTHORIZED
                  SBF_Error = 7;
              otherwise
                  SBF_Error = unknown; % No corresponding OEM7 field, set: no convergence
          end
       */

      if(error == 1 || error == 5 || error == 11 || error == 15)
        return data_types::GNSSSolutionType::NO_SOLUTION;

      switch (type) {
        case 0: {
          return data_types::GNSSSolutionType::NO_SOLUTION;
        }
        case 1: case 2: case 10: {
          return data_types::GNSSSolutionType::SINGLE;
        }
        case 5: case 6: {
          return data_types::GNSSSolutionType::RTKFLOAT;
        }
        case 4: {
          return data_types::GNSSSolutionType::RTKFIX;
        }
        default:
          return data_types::GNSSSolutionType::SINGLE;
      }
    }

}
#endif //ONLINE_FGO_GNSSUTILS_H
