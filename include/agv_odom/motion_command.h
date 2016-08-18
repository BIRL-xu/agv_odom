/*
 * command_fuction.h
 *
 *  Created on: Jul 13, 2016
 *      Author: paul
 */

#ifndef INCLUDE_HANS_AGV_ODOM_COMMAND_FUNCTION_H_
#define INCLUDE_HANS_AGV_ODOM_COMMAND_FUNCTION_H_

#include "agv_odom/data_definition.h"
#include <queue>
namespace HANsAGV
{
        void driveCommand(int left_motor_speed, int right_motor_speed, ControlPackge & cmd_package);
        int linearInterpolation(const int expected_value, const int feedback_value, const int num_samples, int samples[]);

        //滑动均值滤波。
        int moveAverageFilter(int *filter_buffer,  const int expected_value, const int buffer_size);
        void optimMoveAverageFilter(std::queue<int> *filter_buffer,
                                                            const int expected_value_left, const int expected_value_right,
                                                            int &dv_left, int &dv_right,
                                                            const int buffer_size);
        void driveCommand(const float left_motor_speed, const float right_motor_speed,                           //Expected motors' speed.
                                            const float curr_left_motor_speed, const float curr_right_motor_speed,           //Motors' current speed.
                                            ControlPackge & cmd_package);                                                                       //Retrieve a drive command.
        void stopCommand(ControlPackge &stop_cmd);
};

#endif /* INCLUDE_HANS_AGV_ODOM_COMMAND_FUNCTION_H_ */
