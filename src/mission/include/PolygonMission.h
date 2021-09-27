#ifndef POLYGONMISSION_H
#define POLYGONMISSION_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/PositionReference.h"

class PolygonMission{
public:
    PolygonMission();
    ~PolygonMission();
    void init(ros::NodeHandle node);  //  初始化命令、目标点、订阅和发布

    void run(); //  执行状态机

    void polygonplannerCallback(const prometheus_msgs::PositionReference::ConstPtr& msg);
    void dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg);

private:
    prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
    prometheus_msgs::DroneState drone_state_;                                   //无人机状态量
    prometheus_msgs::PositionReference polygon_planner_cmd_;          // fast planner cmd
    
    int mission_cmd_ = 0;//控制状态切换  
    enum  mission_fsm_ {idle, takeoff, hold, move, land, disarm};//高级指令
    int chose_planner_;//选择规划方法
    enum planner_{no_planner, kino, polygon};
    
    // int waypoint_num_;  double waypoints_[50][3];   //  目标点
    std::vector<Eigen::Vector3d> goalpoints_; int goalsize_;
    Eigen::Vector3d goal_;
    double takeoff_height_;//用户自定义起飞高度，大于0.5生效
    Eigen::Vector3d home_position_;//解锁时，记录无人机相对home位置
    double hover_duration_, hover_max_;//悬停时间
    
    bool ishold, ismove, isSendGoal;
    bool hasPlanningPoints_;
    double holdtime_start, holdtime;

    // double move_start, planning_start;

    ros::Subscriber drone_state_sub, polygon_planner_sub;
    ros::Publisher command_pub, goal_pub;
    
    bool ModeChange(prometheus_msgs::ControlCommand cmd, int mode);

};

#endif
