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

#ifndef ONLINE_FGO_GRAPHUTILS_H
#define ONLINE_FGO_GRAPHUTILS_H

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <gtsam/linear/NoiseModel.h>
#include "data/DataTypes.h"

namespace fgo::graph
{

    /***
     *
     * @param timeStatePairs
     * @param timeToQuery
     * @return
     */
    inline fgo::data_types::State queryCurrentPredictedState(const std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timeStatePairs,
                                                              const double& timeToQuery)
    {
      auto itAfter = std::lower_bound(timeStatePairs.begin(), timeStatePairs.end(), timeToQuery,
                                      [](const std::pair<rclcpp::Time, fgo::data_types::State> &pair, double timestamp) -> bool {
                                          // true, ... true, true, false(HERE), false, ... false
                                          return pair.first.seconds() <= timestamp;
                                      });
      if(itAfter == timeStatePairs.begin())
          return itAfter->second;
      else
        return (itAfter - 1)->second;
    }

    /***
     *
     * @param modeType
     * @param variance
     * @param robustParam
     * @param factor
     * @return
     */
    inline gtsam::SharedNoiseModel assignNoiseModel(fgo::data_types::NoiseModel modeType,
                                                    const gtsam::Vector& variance,
                                                    double robustParam,
                                                    const std::string& factor = "")
    {
      gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Variances(variance);
      switch (modeType)
      {
        case fgo::data_types::NoiseModel::GAUSSIAN:
        {
          return model;
        }
        case fgo::data_types::NoiseModel::CAUCHY:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustParam), model);
        }
        case fgo::data_types::NoiseModel::HUBER:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustParam), model);
        }
        case fgo::data_types::NoiseModel::DCS:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustParam), model);
        }
        case fgo::data_types::NoiseModel::Tukey:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustParam), model);
        }
        case fgo::data_types::NoiseModel::GemanMcClure:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustParam), model);
        }
        case fgo::data_types::NoiseModel::Welsch:
        {
          return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Welsch::Create(robustParam), model);
        }
        default:
        {
          RCLCPP_WARN(rclcpp::get_logger("gnss_fgo"), "UNKNOWN noise model for factor %s", factor.c_str());
          return model;
        }
      }

    }



}
#endif //ONLINE_FGO_GRAPHUTILS_H
