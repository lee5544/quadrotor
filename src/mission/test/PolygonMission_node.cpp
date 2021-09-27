#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "ros/ros.h"

#include "PolygonMission.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "polygonMission_node");
    ros::NodeHandle nh("~");


    double frequency = 50;
    ros::Rate rate(frequency);

    ROS_INFO("polygonMission_node");
    PolygonMission polygonMission;
    polygonMission.init(nh);

    // wait for FCU connection
    while(ros::ok()){
        polygonMission.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}