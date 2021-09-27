/**
 * @Name:        StatesToFcu 
 * @Author:       yong
 * @Date: 2021-09-26 16:24:44
 * @LastEditors:         yong
 * @LastEditTime:        Do not edit
 * @Description:      发送外部位姿（视觉里程计、gazebo）至PX4飞控
 * @Subscriber:       
 *      nav_msgs::Odometry
 * @Publisher:      
 *      geometry_msgs::PoseStamped
 */
#ifndef STATESTOFCU_H
#define STATESTOFCU_H

#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"


class StatesToFcu{
    public:    
        StatesToFcu(ros::NodeHandle node);
        ~StatesToFcu();

        void run();//按照一定频率发送位姿

        void t265Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void gazeboCallback(const nav_msgs::Odometry::ConstPtr &msg);
        
        void timerCallback(const ros::TimerEvent &e);

    private:
        std::string input_source_;
        Eigen::Vector3f pos_offset;
        float yaw_offset = 0;

        geometry_msgs::PoseStamped t256_vision_, gazebo_;
        
        //---------------------------------------T265------------------------------------------
        Eigen::Vector3d pos_t265;
        Eigen::Quaterniond q_t265;
        Eigen::Vector3d Euler_t265;
        //---------------------------------------gazebo真值相关------------------------------------------
        Eigen::Vector3d pos_gazebo;
        Eigen::Quaterniond q_gazebo;
        Eigen::Vector3d Euler_gazebo;


        ros::Publisher vision_pub;  ros::Subscriber t265_sub;
        ros::Publisher gazebo_pub;  ros::Subscriber gazebo_sub;
        ros::Timer timer ;
};

#endif