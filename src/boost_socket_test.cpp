/*
 * boost_socket_test.cpp
 *
 *  Created on: Jul 29, 2016
 *      Author: paul
 */
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "agv_odom/clientsocket.h"
#include <boost/thread/thread.hpp>
#include <agv_odom/data_handle.h>
#include <agv_odom/motion_command.h>
#include <agv_odom/HANsAGVMotionConfig.h>
class BoostSocketTest
{
public:
  BoostSocketTest()
{

    try
    {

//      tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string("192.168.1.32"), 2000);
      tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string("127.0.0.1"), 2000);
      datahandle_ptr = boost::shared_ptr<DataHandle>(new DataHandle(endpoint));

      dsrv_ = new dynamic_reconfigure::Server<agv_odom::HANsAGVMotionConfig>(ros::NodeHandle("~"));
      dynamic_reconfigure::Server<agv_odom::HANsAGVMotionConfig>::CallbackType cb = boost::bind(&BoostSocketTest::reconfigureCB, this, _1);
      dsrv_->setCallback(cb);

      timer_ = ros_h.createTimer(ros::Duration(0.05), boost::bind(&BoostSocketTest::send, this));
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}
  ~BoostSocketTest()
  {

  }
  boost::shared_ptr<DataHandle> datahandle_ptr;
private:
  void reconfigureCB(agv_odom::HANsAGVMotionConfig &config);
  void send();
private:

//  boost::asio::io_service::work work;
  boost::asio::io_service io_service;

  ros::NodeHandle ros_h;
  ros::Publisher vel_pub_;
  ros::Timer timer_;
  dynamic_reconfigure::Server<agv_odom::HANsAGVMotionConfig> *dsrv_;
  agv_odom::HANsAGVMotionConfig default_config_;
  bool stop_cmd_;
  bool setup_;
  float vel_x_, vel_w_;
  int left_motor_speed_, right_motor_speed_;
};
void BoostSocketTest::reconfigureCB(agv_odom::HANsAGVMotionConfig &config)
{
      if (setup_ && config.restore_defaults)
      {
          config = default_config_;
          config.restore_defaults = false;
      }
      if ( ! setup_)
      {
          default_config_ = config;
          setup_ = true;
      }
      default_config_ = config;
      vel_x_ = config.linear_vel;
      vel_w_ = config.angular_vel;

      left_motor_speed_ = config.left_motor_speed;
      right_motor_speed_ = config.right_motor_speed;

      stop_cmd_ = config.stop;
      std::cout << "dynamic" << std::endl;
}

void BoostSocketTest::send()
{
  if(stop_cmd_)
    datahandle_ptr->move(0, 0);
  else
 //   datahandle_ptr->move(left_motor_speed_, right_motor_speed_);              //wheel speed.
    datahandle_ptr->motorTest(left_motor_speed_, right_motor_speed_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boost_socket_test_node");
    ros::NodeHandle n;

    BoostSocketTest boost_test;
    boost_test.datahandle_ptr->motorTest(100, -100);

 //   FeedbackData data = boost_test.datahandle_ptr->getFeedbackData();

    ROS_INFO("ros loop");
    ros::Rate looprate(20);
    while(n.ok())
    {
 //     ROS_INFO("main");
 //     std::cout << "left_motor=" << data.motors_speed.left_motor << std::endl;
      ros::spinOnce();
      looprate.sleep();
    }

    boost_test.datahandle_ptr->motorTest(0, 0);
//    ros::spin();
    return 0;
}


