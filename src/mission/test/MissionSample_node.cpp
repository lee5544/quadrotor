#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "ros/ros.h"

#include "MissionSample.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MissionSample");
    ros::NodeHandle nh("~");


    double frequency = 50;
    ros::Rate rate(frequency);

    ROS_INFO("MissionSample");
    MissionSample missionSample;
    missionSample.init(nh);

    // wait for FCU connection
    while(ros::ok()){
        missionSample.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}