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

// TODO @Haoming

#ifndef ONLINE_FGO_GPEXTRAPOLATORPOSE3WNOJ_H
#define ONLINE_FGO_GPEXTRAPOLATORPOSE3WNOJ_H

#pragma once

#include "models/gp_interpolator/GPtemp.h"
#include "utils/GPutils.h"


namespace fgo {
    namespace models {
        class GPExtrapolatorPose3WNOJ {

            private:
                typedef GPExtrapolatorPose3WNOJ This;
                fgo::utils::Matrix_18 Phi_;

            public:
                /// Default constructor: only for serialization
                GPExtrapolatorPose3WNOJ() {}

                GPExtrapolatorPose3WNOJ(double tau){
                    // Calcuate phi
                    Phi_ = fgo::utils::calcPhi3<6>(tau);
                }

            /** Virtual destructor */
            virtual ~GPExtrapolatorPose3WNOJ() {}

            /// extrapolate pose and velocity
            gtsam::NavState extrapolateState(const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n,
                                             const gtsam::Vector3 &omega1_b, const gtsam::Vector3 &acc1_b,
                                             const gtsam::Vector3 &gacc1_b) {
              gtsam::Vector6 acc1 = fgo::utils::convertGbAbtoGbAw(gacc1_b, acc1_b, pose1);
              gtsam::Vector6 vel1 = (gtsam::Vector6() << omega1_b, v1_n).finished();
              const fgo::utils::Vector_18 r1 = (fgo::utils::Vector_18() << gtsam::Vector6::Zero(), vel1, acc1).finished();
              gtsam::Pose3 relPose = gtsam::Pose3::Expmap(Phi_.block<6, 18>(0, 0) * r1);
              gtsam::Pose3 estPose(pose1.rotation()*relPose.rotation(), pose1.translation() + relPose.translation());
              gtsam::Vector3 estVel = Phi_.block<3, 18>(9, 0) * r1;
              gtsam::Vector3 estOmega = Phi_.block<3, 18>(6, 0) * r1;
              gtsam::NavState extrapolatedState = gtsam::NavState(estPose,estVel);
              return extrapolatedState;
            }

            /**
             * Testables
             */

            /** equals specialized to this factor */
            virtual bool equals(const This &expected, double tol = 1e-9) const {
                return gtsam::equal_with_abs_tol(this->Phi_, expected.Phi_, tol);
            }

            /** print contents */
            void print(const std::string &s = "") const {
                std::cout << s << "GaussianProcessExtrapolatorPose3WNOJ" << std::endl;
                std::cout << "phi = " << Phi_ << std::endl;
            }

        private:
            /** Serialization function */
            friend class boost::serialization::access;

            template<class ARCHIVE>
            void serialize(ARCHIVE &ar, const unsigned int version) {
                using namespace boost::serialization;
                ar & make_nvp("Phi", make_array(Phi_.data(), Phi_.size()));
            }

        };
    }
}

#endif //ONLINE_FGO_GPEXTRAPOLATORPOSE3WNOJ_H
