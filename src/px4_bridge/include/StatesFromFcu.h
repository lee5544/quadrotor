/**
 * @Name:         StatesFromFcu
 * @Author:       yong
 * @Date: 2021-09-26 19:29:05
 * @LastEditors:         yong
 * @LastEditTime:        Do not edit
 * @Description:      把从PX4飞控读取无人机状态和位姿，整合至消息DroneState中
 * @Subscriber:       
 *      无人机状态: mavros_msgs::State
 *      无人机位姿: nav_msgs::Odometry
 * @Publisher:    
 *      prometheus_msgs::DroneState    
 */
#ifndef STATESFROMFCU_H
#define STATESFROMFCU_H

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include <prometheus_msgs/DroneState.h>

// #include  "global_var.h"

class StatesFromFcu
{
public:
    //constructed function
    StatesFromFcu();
    ~StatesFromFcu();        

    void init(ros::NodeHandle& state_nh);

    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void localOdomCallback(const nav_msgs::OdometryConstPtr &msg);

private:   
    //变量声明 
    prometheus_msgs::DroneState drone_state_;

    ros::NodeHandle state_nh;
    ros::Subscriber state_sub;
    ros::Subscriber odom_sub;
    ros::Publisher state_pub;

    Eigen::Vector3d quaternion_to_euler(geometry_msgs::Quaternion &q);
};

#endif
