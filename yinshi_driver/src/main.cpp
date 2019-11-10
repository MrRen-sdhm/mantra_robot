#include "ros/ros.h"
#include "yinshi_driver/HandControl.h"

#include "../include/yinshi_driver.h"
#include "../include/serialport.h"

#include <unistd.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yinshi_driver");
    ros::NodeHandle nh;

    YinShiDriver yinShiDriver(&nh, 1);

    ros::spin();

    return 0;
}


