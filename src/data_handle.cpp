/*
 * data_handle.cpp
 *
 *  Created on: Aug 5, 2016
 *      Author: paul
 */
#include <agv_odom/data_handle.h>
#include <agv_odom/motion_command.h>
#include <fstream>      //读写文件流的头文件．
boost::shared_mutex mutex;                       //use a extern variable.
//boost::mutex read_mutex;
std::ofstream time_file;
FeedbackData DataHandle::feedback_data = {{0,0}, {0,0}, {0,0}, {0,0}};      //Initializing a static type struct member.
DataHandle::DataHandle(tcp::endpoint &endpoint)
{
      time_file.open("/home/paul/data/time.txt");
     clientsoc_ptr_ =  boost::shared_ptr<ClientSocket>(new ClientSocket(io_service, endpoint, buffer_size));
 //  boost::shared_ptr<boost::thread> read_thrd_ptr(new boost::thread(boost::bind(&ClientSocket::socketRead, clientsoc_ptr_, recv_buffer_, boost::ref(recv_bytes_))));  //quickly read thread.
     sample_thrd_ptr_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DataHandle::getBufferSample, this)));              //the thread of sampling the buffer.
}
DataHandle::~DataHandle()
{
 //   sample_thrd_ptr->release_handle();
}

void DataHandle::dataTest()
{
  while(true)
  {
    std::cout << "receive data bytes =" << clientsoc_ptr_->recv_bytes_ << "" << std::endl;
    sleep(1);
    std::cout << "buffer first element is" << clientsoc_ptr_->recv_buffer_[0] << std::endl;
  }
}

void DataHandle::getBufferSample()
{

    static long unsigned int counter = 1;
    while(true)
    {
        wait(60);                       //40ms -> 25HZ.

        boost::shared_lock<boost::shared_mutex> lock(mutex);                                     //threads read only can use a shared clock.
 //       read_mutex.lock();
        if(!clientsoc_ptr_->recv_bytes_)            //empty buffer cache.
          continue;
        else                              //we get data.
        {
            int n = clientsoc_ptr_->recv_bytes_ / 34;                                                           //34 is included in a packet defined by custom protocol.Thus, we can get the packet numbers received by the socket.
    //        std::cout << "n= " <<  n<< std::endl;
            if(n == 0 || n == clientsoc_ptr_->recv_bytes_ )                                           //we get a incomplete package and the recv_bytes_ < 34, we must deal with this exception.
            {
                std::cout << "incompete";
 //               read_mutex.unlock();
                continue;
            }
    /*        else if((recv_bytes_ % 34) != 0 && n == 1)       //In this situation, we get the bytes more than 34, but less than 68,that is glue bag phenomenon.
            {
                //we just can get one whole packet.
            }
            else if((recv_bytes_ % 34) != 0 && n > 1)       //In this situation, we also have a problem with glue bag,
            {
                //we can get two packets at least.
            }*/
            else                                                                    //we get all complete packets in the buffer.
            {
                //we can get arbitrary packet in the buffer. Of course, we generally get the last one because we need the newest data.
     // [the packet's header]
               if((unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 34] == 170 &&      //condition1:AA?
                  (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 32] == 30)         //  condition2:1E?
              {
                read_agv_.FrameDataHeader.FrameHeader[0] = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 34];
                read_agv_.FrameDataHeader.FrameHeader[1] = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 33];
                read_agv_.FrameDataHeader.DatasLength = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 32];

       // [[data type 1]]
                read_agv_.SensorsData.MsgHeader = clientsoc_ptr_->recv_buffer_[n*34 - 31];
                read_agv_.SensorsData.MsgLength = clientsoc_ptr_->recv_buffer_[n*34 - 30];

                //time stamp
                read_agv_.SensorsData.TimeStamp[0] = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 29];
                read_agv_.SensorsData.TimeStamp[1] = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 28];

                //reserved sensor data bytes
                read_agv_.SensorsData.Bumper = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 27];
                read_agv_.SensorsData.WheelMissed = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 26];
                read_agv_.SensorsData.TouchGround = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 25];

                //[IMPORTANT]:two encoders' data
                //left encoder
                read_agv_.SensorsData.WheelLeft_Encoder[0] = clientsoc_ptr_->recv_buffer_[n*34 - 24];
                read_agv_.SensorsData.WheelLeft_Encoder[1] = clientsoc_ptr_->recv_buffer_[n*34 - 23];
                read_agv_.SensorsData.WheelLeft_Encoder[2] = clientsoc_ptr_->recv_buffer_[n*34 - 22];
                read_agv_.SensorsData.WheelLeft_Encoder[3] = clientsoc_ptr_->recv_buffer_[n*34 - 21];

                //right encoder
                read_agv_.SensorsData.WheelRight_Encoder[0] = clientsoc_ptr_->recv_buffer_[n*34 - 20];
                read_agv_.SensorsData.WheelRight_Encoder[1] = clientsoc_ptr_->recv_buffer_[n*34 - 19];
                read_agv_.SensorsData.WheelRight_Encoder[2] = clientsoc_ptr_->recv_buffer_[n*34 - 18];
                read_agv_.SensorsData.WheelRight_Encoder[3] = clientsoc_ptr_->recv_buffer_[n*34 - 17];

                //reserved data bytes
                read_agv_.SensorsData.PWMLeft = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 16];
                read_agv_.SensorsData.PWMRight = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 15];
                read_agv_.SensorsData.HANButton = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 14];
                read_agv_.SensorsData.HANCharger = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 13];
                read_agv_.SensorsData.HANBattery = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 12];
                read_agv_.SensorsData.CurrentBeyond = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 11];

        //[[data type 2]]
                read_agv_.Gyromsg.MsgHeader = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 10];
                read_agv_.Gyromsg.MsgLength = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 9];
                //angle
                read_agv_.Gyromsg.Angle[0] = clientsoc_ptr_->recv_buffer_[n*34 - 8];
                read_agv_.Gyromsg.Angle[1] = clientsoc_ptr_->recv_buffer_[n*34 - 7];
                //angle rate
                read_agv_.Gyromsg.AngleRate[0] = clientsoc_ptr_->recv_buffer_[n*34 - 6];
                read_agv_.Gyromsg.AngleRate[1] = clientsoc_ptr_->recv_buffer_[n*34 - 5];
                //reserved data bytes
                read_agv_.Gyromsg.NULL1 = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 4];
                read_agv_.Gyromsg.NULL2 = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 3];
                read_agv_.Gyromsg.NULL3 = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 2];
  //[whole packet checksum]
                read_agv_.checksum = (unsigned char)clientsoc_ptr_->recv_buffer_[n*34 - 1];
//                     read_mutex.unlock();
                if(counter == 1)
                {
                   prev_calcdata_.gyro_angle = char2Int(read_agv_.Gyromsg.Angle);                //角度需要设置。
                   prev_calcdata_.gyro_anglerate = char2Int(read_agv_.Gyromsg.AngleRate);
                   std::cout << "1st gyro angle = " << prev_calcdata_.gyro_angle << std::endl;
                   std::cout << "1st gyro angle rate = " << prev_calcdata_.gyro_anglerate << std::endl;


                   prev_calcdata_.timestamp = char2Int((char*)read_agv_.SensorsData.TimeStamp);
                   prev_calcdata_.leftmotor_pulse = char4Int(read_agv_.SensorsData.WheelLeft_Encoder);
                   prev_calcdata_.rightmotor_pulse = char4Int(read_agv_.SensorsData.WheelRight_Encoder);

           //        data_mutex_.lock();
                   DataHandle::feedback_data.pulse.left_encoder = prev_calcdata_.leftmotor_pulse;
                   DataHandle::feedback_data.pulse.right_encoder = prev_calcdata_.rightmotor_pulse;

                   DataHandle::feedback_data.gyro.angle = prev_calcdata_.gyro_angle;
                   DataHandle::feedback_data.gyro.angle_rate = prev_calcdata_.gyro_anglerate;
          //         data_mutex_.unlock();
                }
                else
                {
                    //calculation
                  calcdata_.gyro_angle = char2Int(read_agv_.Gyromsg.Angle);
                  calcdata_.gyro_anglerate = char2Int(read_agv_.Gyromsg.AngleRate);
//                          std::cout << "**********angle********"<< std::endl;
//                          std::cout << "anlge = " << calcdata.gyro_angle << std::endl;;
//                          std::cout << "angle rate = " << calcdata.gyro_anglerate << std::endl;

                  calcdata_.timestamp = char2Int((char*)read_agv_.SensorsData.TimeStamp);
//                         std::cout << "timestamp(ms):  " << calcdata.timestamp << std::endl;

                  //需要脉冲计数溢出处理
                  calcdata_.leftmotor_pulse = char4Int(read_agv_.SensorsData.WheelLeft_Encoder);
                  calcdata_.rightmotor_pulse = char4Int(read_agv_.SensorsData.WheelRight_Encoder);
                  int left_enco_diff = calcdata_.leftmotor_pulse - prev_calcdata_.leftmotor_pulse;
                  int right_enco_diff = calcdata_.rightmotor_pulse - prev_calcdata_.rightmotor_pulse;

                  //generating motor speed.需要滤波？
                  int dt_temp = calcdata_.timestamp - prev_calcdata_.timestamp;
                  dt_temp = (dt_temp < 0) ? (dt_temp + 65535) : dt_temp;
//                     const float dt = (float)(calcdata.timestamp - prev_calcdata.timestamp)  / 1000;              //ms -> s.
                  const float dt = (float)dt_temp / 1000.0;
                  std::cout << "dt = " << dt <<std::endl;
                  //写入文件，导入excel看曲线，分析数据。
                  time_file << dt_temp << " " << left_enco_diff << " " << right_enco_diff << std::endl;

         //         data_mutex_.lock();
                  DataHandle::feedback_data.motors_speed.left_motor = (int)(calcdata_.leftmotor_pulse - prev_calcdata_.leftmotor_pulse) / (motor2encoder * dt) * 60;              // motor speed.(RPM)
                  DataHandle::feedback_data.motors_speed.right_motor = (int)(calcdata_.rightmotor_pulse - prev_calcdata_.rightmotor_pulse) / (motor2encoder * dt) * 60;
                  std::cout << "**********motor speed********"<< std::endl;
                  std::cout << "left motor speed=: " <<  DataHandle::feedback_data.motors_speed.left_motor <<std::endl;
                  std::cout << "right motor speed=: " <<  DataHandle::feedback_data.motors_speed.right_motor << std::endl;

                  DataHandle::feedback_data.wheels_speed.left_wheel = (int)(calcdata_.leftmotor_pulse - prev_calcdata_.leftmotor_pulse) / (wheel2motor * dt) * 60;              //wheel speed(RPM), maybe need rad/s.
                  DataHandle::feedback_data.wheels_speed.right_wheel = (int)(calcdata_.rightmotor_pulse- prev_calcdata_.rightmotor_pulse) / (wheel2motor * dt) * 60;

                  DataHandle::feedback_data.pulse.left_encoder = calcdata_.leftmotor_pulse;
                  DataHandle::feedback_data.pulse.right_encoder = calcdata_.rightmotor_pulse;

                  DataHandle::feedback_data.gyro.angle = calcdata_.gyro_angle;
                  DataHandle::feedback_data.gyro.angle_rate = calcdata_.gyro_anglerate;
         //         data_mutex_.unlock();
                  //update
                  prev_calcdata_ = calcdata_;

                }

              }
          }
      }
      counter++;
    }
}
//上层定时调用该函数。
void DataHandle::move(const float left_wheel_speed, const float right_wheel_speed)
{
      //we need transform from rad/s to roll/min.
    const float left_wheel = left_wheel_speed * (M_PI / 30);
    const float right_wheel  = right_wheel_speed * (M_PI / 30);

    //then, we need also transform from wheels_speed' speed to motors_speed' speed.(RPM)
    const int left_motor = (int)(left_wheel * wheel2motor);
    const int right_motor = (int)(right_wheel * wheel2motor);

    //packge a drive command.
    ControlPackge move_cmd;
    HANsAGV::driveCommand(left_motor, right_motor, move_cmd);

    //then, send the command by TCP socket.
    char drive_cmd[sizeof(move_cmd)];
    std::memcpy((char*)drive_cmd, (ControlPackge*)&move_cmd, sizeof(move_cmd));
    clientsoc_ptr_->socketWrite(drive_cmd, sizeof(move_cmd));           //send command.
}

void DataHandle::motorTest(const int left, const int right)
{
    ControlPackge cmd;
    HANsAGV::driveCommand(left, right, cmd);
    char drive_cmd[sizeof(cmd)];
    std::memcpy((char*)drive_cmd, (ControlPackge*)&cmd, sizeof(cmd));
    clientsoc_ptr_->socketWrite(drive_cmd, sizeof(cmd));           //send command.
}

int DataHandle::char4Int(const char* ch)
{
    union
    {
        char bitNum[4];
        int result;
    }ex;

    for(char i = 0; i < 4; i++)
      ex.bitNum[i] = *(ch + i);

    return ex.result;
}

short int DataHandle::char2Int(const char* ch)
{
    union
    {
        char bitNum[2];
        int result;
    }ex;
    for(char i = 0; i < 4; i++)
      ex.bitNum[i] = *(ch + i);
    return ex.result;
}

const FeedbackData &DataHandle::getFeedbackData()
{
  boost::shared_lock<boost::shared_mutex> lock(mutex);
  return DataHandle::feedback_data;
}
const FeedbackData::MotorsSpeed &DataHandle::getMotorsSpeed()
{
  boost::shared_lock<boost::shared_mutex> lock(mutex);
  return DataHandle::feedback_data.motors_speed;
}
const FeedbackData::WheelsSpeed &DataHandle::getWheelsSpeed()
{
  boost::shared_lock<boost::shared_mutex> lock(mutex);
  return DataHandle::feedback_data.wheels_speed;
}

const FeedbackData::Pulse &DataHandle::getEncoderPulse()
{
  boost::shared_lock<boost::shared_mutex> lock(mutex);
  return DataHandle::feedback_data.pulse;
}

const FeedbackData::Gyro &DataHandle::getGyroData()
{
  boost::shared_lock<boost::shared_mutex> lock(mutex);
  return DataHandle::feedback_data.gyro;
}
