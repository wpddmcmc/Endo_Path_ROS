#include "ros/ros.h"
#include "detector/Detector_Info.h"
#include <iostream>
#include <iomanip>

void chatterCallback(const detector::Detector_Info & msg)
{

    std::cout<<"("<<msg.center_x<<","<<msg.center_y<<")"<<std::endl;
 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cavity_data", 1000, chatterCallback);
    ROS_INFO("Listener Start to Monitor");
    ros::spin();

    return 0;
}
