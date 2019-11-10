//
// Created by MrRen-sdhm on 19-11-10.
//

#include "../include/yinshi_driver.h"

#include <cstdlib>
#include <iostream>

#define CMD_MC_PARA_SAVE 0x01
#define CMD_MC_PARA_ID_SET 0x04
#define CMD_MC_MOVE_CATCH_XG 0x18
#define CMD_MC_MOVE_RELEASE 0x11
#define CMD_MC_MOVE_STOPHERE 0x16
#define CMD_MC_SET_EG_PARA 0x12
#define CMD_MC_READ_EG_PARA 0x13
#define CMD_MC_READ_EG_STATE 0x14
#define CMD_ZERO_CALI 0x40
#define CMD_MC_READ_EG_RUNSTATE 0x41

#define FRAME_HEAD_H 0xEB
#define FRAME_HEAD_L 0x90

YinShiDriver::YinShiDriver(ros::NodeHandle* nh, uint8_t id) : nh_(*nh), HandID(id) {
    std::string serialDeviceName_ = "/dev/ttyUSB0";
//    serialport_ = new LinuxSerialPort::SerialPort(serialDeviceName_, LinuxSerialPort::BaudRate::B_115200);
//    serialport_->Open();
    serialdriver_ = new SerialDriver();
    serialdriver_->OpenPort(serialDeviceName_, 115200);

    hand_ctl_service = nh_.advertiseService("hand_control", &YinShiDriver::HandControl, this);
    ROS_INFO("Yinshi hand control service started!");
}

YinShiDriver::~YinShiDriver() {
//    serialport_->Close();
    serialdriver_->ClosePort();
}

bool YinShiDriver::HandControl(yinshi_driver::HandControl::Request  &req, yinshi_driver::HandControl::Response &res) {
    if(req.command == "Open") {
        Move(YinShiDriver::HandCMD::OpenHand, 1000, 1000);
        res.success = true;
    }
    else if(req.command == "Close") {
        Move(YinShiDriver::HandCMD::CloseHand, 1000, 1000);
        res.success = true;
    }
    else {
        printf("Supported command: Open/Close\n");
        res.success = false;
    }

    return true;
}

void YinShiDriver::Move(HandCMD _CMD, int _Speed, int _Cur) {

    std::vector<unsigned char> _Data;
    if(_CMD == CloseHand) {
        _Data.push_back(FRAME_HEAD_H);
        _Data.push_back(FRAME_HEAD_L);
        _Data.push_back((HandID & 0xff));
        _Data.push_back(5);
        _Data.push_back(CMD_MC_MOVE_CATCH_XG);
        _Data.push_back((_Speed % 256) & 0xff);
        _Data.push_back((_Speed / 256) & 0xff);
        _Data.push_back((_Cur % 256) & 0xff);
        _Data.push_back((_Cur / 256) & 0xff);
        char _CheckSum = FrameCheck(_Data);
        _Data.push_back(_CheckSum);
    } else {
        _Data.push_back(FRAME_HEAD_H);
        _Data.push_back(FRAME_HEAD_L);
        _Data.push_back((HandID & 0xff));
        _Data.push_back(3);
        _Data.push_back(CMD_MC_MOVE_RELEASE);
        _Data.push_back((_Speed % 256) & 0xff);
        _Data.push_back((_Speed / 256) & 0xff);
        char _CheckSum = FrameCheck(_Data);
        _Data.push_back(_CheckSum);
    }

//    serialport_->Write(_Data.data(), _Data.size());
    serialdriver_->Write(_Data.data(), _Data.size());
}

unsigned char YinShiDriver::FrameCheck(std::vector<unsigned  char> _Data) {

    char _CheckSum = 0;
    for(int ii=2;ii<_Data.size();ii++)
        _CheckSum += _Data[ii];
    return _CheckSum;

}

void YinShiDriver::ClearError() {

    std::vector<unsigned char> _Data;

    _Data.push_back(FRAME_HEAD_H);
    _Data.push_back(FRAME_HEAD_L);
    _Data.push_back(HandID & 0xff);
    _Data.push_back(1);
    _Data.push_back(0x17);
    char _CheckSum = FrameCheck(_Data);
    _Data.push_back(_CheckSum);
//    serialport_->Write(_Data.data(), _Data.size());
    serialdriver_->Write(_Data.data(), _Data.size());
}