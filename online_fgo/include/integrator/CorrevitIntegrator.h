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

#ifndef ONLINE_FGO_CORREVITINTEGRATOR_H
#define ONLINE_FGO_CORREVITINTEGRATOR_H

#include <irt_nav_msgs/msg/correvit.hpp>
#include <irt_nav_msgs/msg/correvit_pitch_roll.hpp>
#include <irt_nav_msgs/msg/correvit_vel_angle.hpp>

#include "IntegratorBase.h"
#include "factor/measurement/odometry/NavVelocityFactor.h"
#include "factor/measurement/odometry/GPInterpolatedNavVelocityFactor.h"
#include "factor/motion/VelocityPreintegration.h"
#include "factor/measurement/odometry/PreintegratedVelocityFactor.h"
#include "factor/measurement/odometry/GPInterpolatedPreintegratedVelocityFactor.h"

namespace fgo::integrator
{
    struct CorrevitVelAngle
    {
        double timestamp{};
        double angle{};
        double vel{};
        double vel_x{};
        double vel_y{};
        double delay = 0.;
    };

    struct Correvit
    {
        double timestamp{};
        double angle_correvit{};
        double vel_x_correvit{};
        double vel_correvit{};
        double vel_y_correvit{};
        double delay = 0.;
    };

    struct CorrevitPitchRoll
    {
        double timestamp{};
        double pitch{};
        double radius{};
        double roll{};
        double delay = 0.;
    };

    class CorrevitIntegrator : public IntegratorBase
    {
    protected:
        rclcpp::Subscription<irt_nav_msgs::msg::CorrevitVelAngle>::SharedPtr subCorrevitVelAngle_;
        rclcpp::Subscription<irt_nav_msgs::msg::Correvit>::SharedPtr subCorrevit_;
        rclcpp::Subscription<irt_nav_msgs::msg::CorrevitPitchRoll>::SharedPtr subCorrevitPitchRoll_;

        fgo::buffer::CircularDataBuffer<CorrevitVelAngle> bufferCorrevitVelAngle_;
        fgo::buffer::CircularDataBuffer<Correvit> bufferCorrevit_;
        fgo::buffer::CircularDataBuffer<CorrevitPitchRoll> bufferCorrevitPitchRoll_;

        IntegratorCorrevitParamsPtr paramPtr_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolator_;
        std::shared_ptr<fgo::factor::VelocityPreintegrationParams> preIntergratorParams_;

        std::atomic_bool zeroVelocity_ = false;


    public:
        explicit CorrevitIntegrator() = default;
        ~CorrevitIntegrator() override = default;

        void initialize(rclcpp::Node& node,
                        fgo::graph::GraphBase& graphPtr,
                        const std::string& integratorName,
                        bool isPrimarySensor = false) override;

        bool factorize(
            const boost::circular_buffer<std::pair<double, gtsam::Vector3>>& timestampGyroMap,
            const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& currentKeyIndexTimestampMap,
            std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
            gtsam::Values& values,
            fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
            gtsam::KeyVector& relatedKeys
        ) override;

        bool fetchResult(
            const gtsam::Values& result,
            const gtsam::Marginals& martinals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& keyIndexTimestampMap,
            fgo::data_types::State& optState
        ) override;

        void dropMeasurementBefore(double timestamp) override{
          bufferCorrevitVelAngle_.cleanBeforeTime(timestamp);
          bufferCorrevit_.cleanBeforeTime(timestamp);
          bufferCorrevitPitchRoll_.cleanBeforeTime(timestamp);
        }

        bool checkZeroVelocity() override{
          return zeroVelocity_;
        }

        bool checkHasMeasurements() override
        {
          return bufferCorrevit_.size() != 0;
        }

        void cleanBuffers() override
        {
          bufferCorrevit_.clean();
        }


    };
}
#endif //ONLINE_FGO_CORREVITINTEGRATOR_H
