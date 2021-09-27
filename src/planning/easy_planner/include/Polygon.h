#ifndef POLYGON_H
#define POLYGON_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/PositionReference.h"

class Polygon{
public:
    Polygon();
    ~Polygon();
    void init(ros::NodeHandle node);

    void run(int frequency);

    void waypointCallback(const nav_msgs::PathConstPtr& msg);
    void dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg);

private:
    prometheus_msgs::DroneState drone_state_;                                   //无人机状态量
    prometheus_msgs::PositionReference polygon_planner_cmd_;          // fast planner cmd

    bool trigger_;
    Eigen::Vector3d goal_, cur_pos_;
    std::vector<Eigen::Vector3d> traj_;

    ros::Subscriber drone_state_sub_, waypoint_sub_;
    ros::Publisher traj_pub_;

    void polygon_generator(Eigen::Vector3d pos, Eigen::Vector3d goal, int number, std::vector<Eigen::Vector3d> &traj);

};

#endif
