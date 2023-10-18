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

#ifndef FGONAV_GAUSSIANPROCESSEXTRAPOLATORPOSE3VNWB_H
#define FGONAV_GAUSSIANPROCESSEXTRAPOLATORPOSE3VNWB_H

#pragma once

#include "utils/GPutils.h"


namespace fgo {
    namespace models {

/**
 * 4-way factor for Gaussian Process interpolator, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 * interpolate pose and velocity given consecutive poses and velocities
 */
        class GaussianProcessExtrapolatorPose3WNOA {

        private:
            typedef GaussianProcessExtrapolatorPose3WNOA This;
            fgo::utils::Matrix_12 Phi_;

        public:
            /// Default constructor: only for serialization
            GaussianProcessExtrapolatorPose3WNOA() {}

            /**
             * Constructor
             * @param Qc noise model of Qc
             * @param delta_t the time between the two states
             * @param tau the time of interval status
             */
            GaussianProcessExtrapolatorPose3WNOA(double tau){
                // Calcuate phi
                Phi_ = fgo::utils::calcPhi<6>(tau);
            }

            /** Virtual destructor */
            virtual ~GaussianProcessExtrapolatorPose3WNOA() {}

            /// extrapolate pose and velocity
            gtsam::NavState extrapolateState(const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b) {
              gtsam::Vector6 vel1 = (gtsam::Vector6() << omega1_b,v1_n).finished();
              const fgo::utils::Vector_12 r1 = (fgo::utils::Vector_12() << gtsam::Vector6::Zero(), vel1).finished();
              gtsam::Pose3 poseRel = (gtsam::Pose3::Expmap(Phi_.block<6, 12>(0, 0) * r1));
              gtsam::Pose3 pose(pose1.rotation()*poseRel.rotation(), pose1.translation() + poseRel.translation());
              gtsam::Vector3 estVele = poseRel.rotation() * Phi_.block<3, 12>(9, 0) * r1;
              gtsam::Vector3 estOmega = Phi_.block<3, 12>(6, 0) * r1; //could be used but WNOA boring
              gtsam::NavState extrapolatedState(pose,estVele);
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
                std::cout << s << "GaussianProcessExtrapolatorPose3WNOA" << std::endl;
                std::cout << "phi = " << Phi_ << std::endl;
            }

        private:
            /** Serializat ion function */
            friend class boost::serialization::access;

            template<class ARCHIVE>
            void serialize(ARCHIVE &ar, const unsigned int version) {
                using namespace boost::serialization;
                ar & make_nvp("Phi", make_array(Phi_.data(), Phi_.size()));
            }
        };
    } // namespace solver
} // \ namespace fgonav


/// traits
namespace gtsam {
    template<>
    struct traits<fgo::models::GaussianProcessExtrapolatorPose3WNOA> : public Testable<
        fgo::models::GaussianProcessExtrapolatorPose3WNOA> {};
}

#endif //FGONAV_GAUSSIANPROCESSEXTRAPOLATORPOSE3VNWB_H
