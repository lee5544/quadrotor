#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "Polygon.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polygon_planner");
    ros::NodeHandle nh("~");

    double frequency = 10;
    ros::Rate rate(frequency);
    double dt;//dt近似等于1/rate
    dt = 1 / frequency;

    Polygon polygon;
    polygon.init(nh);

    // wait for FCU connection
    while(ros::ok()){
        ros::spinOnce();
        polygon.run(frequency);
        rate.sleep();
    }

    return 0;
}