//
// Created by MrRen-sdhm on 19-11-10.
//

#ifndef YINSHIDRIVER_H
#define YINSHIDRIVER_H

#include "ros/ros.h"
#include "yinshi_driver/HandControl.h"

#include "serialport.h"
#include "serial_driver.h"

class YinShiDriver{
public:
    enum HandCMD{OpenHand, CloseHand};

public:
    YinShiDriver(ros::NodeHandle* nh, uint8_t id);
    ~YinShiDriver();

    void Move(HandCMD _CMD, int _Speed, int _Cur);
    void MovePose(int _Pose);
    void ClearError();

private:
    unsigned char FrameCheck(std::vector<unsigned char> _Data);
    bool HandControl(yinshi_driver::HandControl::Request  &req, yinshi_driver::HandControl::Response &res);

    ros::NodeHandle nh_;
    ros::ServiceServer hand_ctl_service;
//    LinuxSerialPort::SerialPort* serialport_;
    SerialDriver* serialdriver_;

private:
    int HandID = 1;
};


#endif //YINSHIDRIVER_H
