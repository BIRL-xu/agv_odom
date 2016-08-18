/*
 * data_definition.h
 *
 *  Created on: Aug 1, 2016
 *      Author: paul
 */

#ifndef INCLUDE_AGV_ODOM_DATA_DEFINITION_H_
#define INCLUDE_AGV_ODOM_DATA_DEFINITION_H_

//==================================================
/*
 *                                      AGV反馈数据格式
 */
//==================================================

typedef unsigned char CheckSUM;         //校验类型．
typedef unsigned char byte;

typedef struct HANsAGVMsgHeader         //父数据头
{
        unsigned char FrameHeader[2];
        unsigned char DatasLength;
}HANSMSGHEADER;

typedef struct HANsAGVSensorsMsg                //base sensors message.
{
        byte MsgHeader;
        byte MsgLength;
        byte TimeStamp[2];
        byte Bumper;                    //保险杠
        byte WheelMissed;
        byte TouchGround;
        char WheelLeft_Encoder[4];
        char WheelRight_Encoder[4];
        byte PWMLeft;
        byte PWMRight;
        byte HANButton;
        byte HANCharger;
        byte HANBattery;
        byte CurrentBeyond;     //过电流．
}AGVSENSORS;

typedef struct HANsAGVGyroMsg           //陀螺仪传感器数据．
{
        byte MsgHeader;
        byte MsgLength;

        char Angle[2];
        char AngleRate[2];
        byte NULL1;
        byte NULL2;
        byte NULL3;
}GyroMSG;

typedef struct HANsAGVFeedbackData
{
        HANSMSGHEADER FrameDataHeader;
        AGVSENSORS SensorsData;
        GyroMSG Gyromsg;
        CheckSUM checksum;
}READ_AGV;

//=======================================
/*
 *                           发送控制指令
 */
//=======================================

//车轮转速的高/低字节．
union CommandBit
{
        int MotorSpeed;                 //电机的轮速(RPM),注意单位的统一。
        byte NumBit[4];                  //轮速对应的四个字节。
};

struct ControlPackge
{
  HANSMSGHEADER FrameDataHeader;
  struct ControlCommand
  {
          byte DataHeader;                  //控制指令的数据类型编号。
          byte DataLenght;                  //默认数据长度为8个字节。
          byte LeftMotor[4];
          byte RightMotor[4];
  }cmd;
  CheckSUM checksum;
  ControlPackge()
  {
        FrameDataHeader.FrameHeader[0] = 0xAA;
        FrameDataHeader.FrameHeader[1] = 0x55;
        FrameDataHeader.DatasLength = 0x0B;
        cmd.DataHeader = 0x01;
        cmd.DataLenght = 0x08;
        checksum = 0x00;
  }
};

//=============================================
/*
 *                                  计算数据
 */
//=============================================
struct CalcData
{
    unsigned short int timestamp;
    short int gyro_angle;
    short int gyro_anglerate;
    int leftmotor_pulse;
    int rightmotor_pulse;
};
#endif /* INCLUDE_AGV_ODOM_DATA_DEFINITION_H_ */
