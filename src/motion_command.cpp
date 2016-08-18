/*
 * commad_fuction.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: paul
 */
#include <agv_odom/motion_command.h>
#include<iostream>
#include <cstdlib>
namespace HANsAGV
{
/*Function Getting two wheels roll speed (rpm) and packet a drive command based on protocol.
 * param1 Input - The left wheel  speed.(rpm).
 * param2 Input - The right wheel speed. (rpm)
 * param3 Output - The expected drive command.
 * return
 */
      void driveCommand( int left_motor_speed, int right_motor_speed, ControlPackge &cmd_package)
      {
           CommandBit left_motor, right_motor;

           left_motor_speed = (std::abs(left_motor_speed) < 100) ? 0 : left_motor_speed;
           right_motor_speed = (std::abs(right_motor_speed) < 100) ? 0 : right_motor_speed;

           left_motor.MotorSpeed = left_motor_speed;
           right_motor.MotorSpeed = right_motor_speed;
           // Left
           cmd_package.cmd.LeftMotor[0] = left_motor.NumBit[0];
           cmd_package.cmd.LeftMotor[1] = left_motor.NumBit[1];
           cmd_package.cmd.LeftMotor[2] = left_motor.NumBit[2];
           cmd_package.cmd.LeftMotor[3] = left_motor.NumBit[3];
           //right
           cmd_package.cmd.RightMotor[0] = right_motor.NumBit[0];
           cmd_package.cmd.RightMotor[1] = right_motor.NumBit[1];
           cmd_package.cmd.RightMotor[2] = right_motor.NumBit[2];
           cmd_package.cmd.RightMotor[3] = right_motor.NumBit[3];
      }

      //线性插补
      int linearInterpolation(const int expected_value, const int feedback_value, const int num_samples, int samples[])
      {
          int sum = 0;
          if(expected_value == feedback_value)
          {
              for(char i = 0; i <10; i++)
              {
                  samples[i] = expected_value;
                  sum += samples[i];
              }
          }
          else
          {
              int step_size = (expected_value - feedback_value) / num_samples;
              int next = feedback_value;
              for(char j = 0 ; j < num_samples; j++)
              {
                  next +=step_size;
                  samples[j] =next;
                  sum += samples[j];
              }
          }
          return (int)(sum / 10);          //返回插补的平均值。
      }
//滑动均值滤波算法
      int moveAverageFilter(int *filter_buffer,  const int expected_value, const int buffer_size)
      {
          int filter_sum = 0;
          for(char i = 0; i < buffer_size -1; i++)
          {
              filter_buffer[i] = filter_buffer[i+1];
              filter_sum += filter_buffer[i];
          }
          filter_buffer[buffer_size -1] = expected_value;
          filter_sum += filter_buffer[buffer_size -1];
          return (int)(filter_sum / buffer_size);
      }
  //优化滑动均值滤波算法
      void optimMoveAverageFilter(std::queue<int> *filter_buffer,
                                                          const int expected_value_left, const int expected_value_right,
                                                          int &dv_left, int &dv_right,
                                                          const int buffer_size)
      {
          static long int filter_sum1 = 0, filter_sum2 = 0;
          static unsigned int i_counter = 1;
          if(1 == i_counter)
          {
  //            filter_buffer[buffer_size -1] = expected_value;     //替换最后一个值。
              if(filter_buffer[0].size() == buffer_size && filter_buffer[1].size() == buffer_size)
              {
                  filter_buffer[0].pop();                                       //删除第一个元素。
                  filter_buffer[1].pop();
              }
              while((filter_buffer[0].size() < buffer_size))
              {
                  filter_buffer[0].push(expected_value_left);              //添加新元素到队尾
                  filter_buffer[1].push(expected_value_right);              //添加新元素到队尾
              }
              filter_sum1 = filter_buffer[0].back();                 //第一次进入时只有最有一个元素不为0;
              filter_sum2 = filter_buffer[1].back();
          }
          else
          {
              if(filter_buffer[0].empty())
              {
                  std::cout << "Filter buffer is empty!!"<<std::endl;
                  return;
              }
              while(filter_buffer[0].size() > buffer_size)
              {
                  std::cout << "pop!!"<<std::endl;
                  filter_buffer[0].pop();
                  filter_buffer[1].pop();
              }
              while(filter_buffer[0].size() < buffer_size)
              {
 //                   std::cout << "push!!"<<std::endl;
                    filter_buffer[0].push(expected_value_left);              //添加新元素到队尾
                    filter_buffer[1].push(expected_value_right);              //添加新元素到队尾
              }
              if((filter_buffer[0].size() == buffer_size) && (filter_buffer[1].size() == buffer_size)){
//              std::cout << "counter = "<<i_counter << std::endl;
//              std::cout << "front:"<<filter_buffer[0].front()<< std::endl;                 //为什么到第15次才是expected_value;
              filter_sum1 = filter_sum1 + expected_value_left - filter_buffer[0].front();         //front取出乱值。
              filter_sum2 = filter_sum2 + expected_value_right - filter_buffer[1].front();         //front取出乱值。
              std::cout << filter_sum1<< std::endl;}
             filter_buffer[0].pop();                                       //删除第一个元素。
             filter_buffer[1].pop();                                       //删除第一个元素。
          }


//          filter_sum = filter_sum - env + expected_value;
//          env = (int)(filter_sum / buffer_size);
//          std::cout << filter_sum << std::endl;
         i_counter ++;
         dv_left = (int)(filter_sum1 / buffer_size);
         dv_right = (int)(filter_sum2 / buffer_size);
      }

      /*
       * Function Getting the stop command..
       */
      void stopCommand(ControlPackge &stop_cmd)
      {
          driveCommand(0, 0, stop_cmd);
      }
};
