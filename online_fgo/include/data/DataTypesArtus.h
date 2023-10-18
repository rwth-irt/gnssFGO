//
// Created by haoming on 24.06.22.
//

#ifndef ONLINE_FGO_DATATYPESCORREVIT_H
#define ONLINE_FGO_DATATYPESCORREVIT_H

#pragma once

namespace fgo::data_types
{

    struct CorrevitObs
    {
        double            timestamp;
        double            angle_correvit;
        double            vel_x_correvit;
        double            vel_y_correvit;
        double            vel_correvit;
        double            angle;
        double            vel;
        double            vel_x;
        double            vel_y;
        double            distance;
        double            pitch;
        double            radius;
        double            roll;
        double            status;
        double            angle_vel_correction;
        double            angle_switch_off;
        double            filter_on;
        double            filter_settings;
        double            head_status;
        double            lamp_current;
        double            stst;
        double            temperature;
        double            temp_ok;
    };

    struct DumperOdom
    {
        double            timestamp;
        double            wheelspeed_count_fl;
        double            wheelspeed_count_fr;
        double            wheelspeed_count_rl;
        double            wheelspeed_count_rr;
        double            bin_position;
        double            bin_tilt;
        double            boost;
        double            brake;
        double            direction_value;
        double            engine_speed;
        double            fuel;
        double            handbreak;
        double            horn;
        double            steering_angle;
        double            steering_rate;
        double            steering_switch;
        double            throttle;
    };









}
#endif //ONLINE_FGO_DATATYPESCORREVIT_H
