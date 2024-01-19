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

//
// Created by haoming on 07.01.24.
//

#ifndef ONLINE_FGO_MATLAB_UTILS_H
#define ONLINE_FGO_MATLAB_UTILS_H

#include <array>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include "tmwtypes.h"

#include "genJacobianECEF2ENU/genJacobianECEF2ENU.h"
#include "genJacobianECEF2NED/genJacobianECEF2NED.h"


namespace matlab_utils
{
    inline gtsam::Matrix36 jacobianECEF2NED(const gtsam::Vector3& t_ref, const gtsam::Vector3& t)
    {
        creal_T jac[18];
        // double x, double xref, double y, double yref, double z, double zref,
        //  0          1            2            3            4           5
        //  6          7            8            9            10          11
        //  12         13           14           15           16           17
        genJacobianECEF2NED(t.x(), t_ref.x(), t.y(), t_ref.y(), t.z(), t_ref.z(), jac);
        // Jacobian for (t_ref; t)
        return (gtsam::Matrix36() << jac[0].re, jac[1].re, jac[2].re,
                                     jac[3].re, jac[4].re, jac[5].re,
                                     jac[6].re, jac[7].re, jac[8].re,
                                     jac[9].re, jac[10].re, jac[11].re,
                                     jac[12].re, jac[13].re, jac[14].re,
                                     jac[15].re, jac[16].re, jac[17].re).finished();
    }

    inline gtsam::Matrix36 jacobianECEF2ENU(const gtsam::Vector3& t_ref, const gtsam::Vector3& t)
    {
        creal_T jac[18];
        // double x, double xref, double y, double yref, double z, double zref,
        //  0          1            2            3            4           5
        //  6          7            8            9            10          11
        //  12         13           14           15           16           17
        genJacobianECEF2ENU(t.x(), t_ref.x(), t.y(), t_ref.y(), t.z(), t_ref.z(), jac);

        return (gtsam::Matrix36() << jac[0].re, jac[1].re, jac[2].re,
                jac[3].re, jac[4].re, jac[5].re,
                jac[6].re, jac[7].re, jac[8].re,
                jac[9].re, jac[10].re, jac[11].re,
                jac[12].re, jac[13].re, jac[14].re,
                jac[15].re, jac[16].re, jac[17].re).finished();
    }


}
#endif //ONLINE_FGO_MATLAB_UTILS_H
