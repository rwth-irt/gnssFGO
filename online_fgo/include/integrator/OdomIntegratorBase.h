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


#ifndef ONLINE_FGO_ODOMINTEGRATORBASE_H
#define ONLINE_FGO_ODOMINTEGRATORBASE_H

#pragma once

#include "IntegratorBase.h"

namespace fgo::integrator
{
    class OdomIntegratorBase : public IntegratorBase
    {
    protected:
        std::shared_ptr<fgo::models::GPInterpolator> interpolatorI_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolatorJ_;
        IntegratorOdomParamsPtr paramPtr_;
        std::vector<fgo::data_types::OdomResult> odomResults_;

    public:
        explicit OdomIntegratorBase() = default;



    };
}
#endif //ONLINE_FGO_ODOMINTEGRATORBASE_H
