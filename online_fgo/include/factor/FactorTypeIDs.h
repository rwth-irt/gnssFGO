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


#ifndef ONLINE_FGO_FACTORTYPEIDS_H
#define ONLINE_FGO_FACTORTYPEIDS_H

namespace fgo::factor
{
    enum FactorTypeIDs
    {
        CombinedIMU = 0,
        GPWNOJMotionPrior = 1,
        GPWNOAMotionPrior = 2,
        ReceiverClock = 3,
        NavPose = 4,
        GPNavPose = 5,
        NavAttitude = 6,
        GPNavAttitude = 7,
        NavVelocity = 8,
        GPNavVelocity = 9,
        BetweenPose = 10,
        GPDoubleBetweenPose = 11,
        GPSingleBetweenPose = 12,
        GPS = 13,
        GPGPS = 14,
        PVT = 15,
        GPPVT = 16,
        PRDR = 17,
        GPPRDR = 18,
        PR = 19,
        GPPR = 20,
        DR = 21,
        GPDR = 22,
        TDCP = 23,
        GPTDCP = 24,
        DDCP = 25,
        GPDDCP = 26,
        DDPR = 27,
        GPDDPR = 28,
        DDPRDR = 29,
        GPDDPRDR = 30,
        TDCP3 = 31,
        GPTDCP3 = 32,
        ConstVelocity = 33,
        ConstAngularVelocity = 34,
        ConstAcceleration = 35
    };
}



#endif //ONLINE_FGO_FACTORTYPEIDS_H
