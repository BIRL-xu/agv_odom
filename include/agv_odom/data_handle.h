/*
 * data_handle.h
 *
 *  Created on: Aug 4, 2016
 *      Author: paul
 */

#ifndef INCLUDE_AGV_ODOM_DATA_HANDLE_H_
#define INCLUDE_AGV_ODOM_DATA_HANDLE_H_
#include <agv_odom/clientsocket.h>
#include <agv_odom/data_definition.h>
#include <vector>

#define M_PI 3.14159265358979323846

const unsigned int buffer_size = 1700;                               // receive buffer's size.
const unsigned short int wheel2motor = 30;                      //motor's reduction ratio.
const unsigned short int motor2encoder = 30 * 2;             // resolution encoder to motor.30 is encoder's resolution relative to motor.
                                                                                             //2 is a pulse detected with raising edge and falling edge can generate two pulse numbers.
const unsigned short int encoder2wheel = wheel2motor * motor2encoder;      //resolution encoder to wheel.30 * 30 is Motor reduction ratio multiply encoder resolution;


struct FeedbackData
{
    struct MotorsSpeed
    {
        int left_motor;
        int right_motor;
    }motors_speed;

    struct WheelsSpeed
    {
        int left_wheel;
        int right_wheel;
    }wheels_speed;

    struct Pulse
    {
        long int left_encoder;
        long int right_encoder;
    }pulse;

    struct Gyro
    {
        short int angle;
        short int angle_rate;
    }gyro;
};

class DataHandle
{
public:
  DataHandle(tcp::endpoint &endpoint);
  ~DataHandle();
public:
  void dataTest();
  void startThread();
  void getBufferSample();
  void move(const float left_wheel_speed, const float right_wheel_speed);       //rad/s.
  void motorTest(const int left, const int right);

  //shared data.
    static FeedbackData feedback_data;
    const FeedbackData &getFeedbackData();
    const FeedbackData::MotorsSpeed &getMotorsSpeed();
    const FeedbackData::WheelsSpeed &getWheelsSpeed();
    const FeedbackData::Pulse &getEncoderPulse();
    const FeedbackData::Gyro &getGyroData();

private:
  int char4Int(const char*);
  short int char2Int(const char*);

  boost::asio::io_service io_service;
  boost::shared_ptr<ClientSocket> clientsoc_ptr_;
  boost::shared_ptr<boost::thread> sample_thrd_ptr_;
//  boost::mutex data_mutex_;

  READ_AGV read_agv_;                                                //AGV's feedback data.
  CalcData prev_calcdata_, calcdata_;
};

#endif /* INCLUDE_AGV_ODOM_DATA_HANDLE_H_ */
