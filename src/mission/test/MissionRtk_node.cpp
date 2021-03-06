#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "ros/ros.h"

#include "MissionRtk.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MissionRtk");
    ros::NodeHandle nh("~");


    double frequency = 20;
    ros::Rate rate(frequency);

    ROS_INFO("MissionRtk");
    MissionRtk missionRtk;
    missionRtk.init(nh);

    // wait for FCU connection
    while(ros::ok()){
        missionRtk.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}