/**
 * @Name:       CommandToFcuAtt
 * @Author:       yong
 * @Date: 2021-11-24 
 * @LastEditors:         yong
 * @LastEditTime:        Do not edit
 * @Description:      解析无人机高级指令，发送期望控制量，实现PX4底层控制
 * @参考: gly   
 */
#ifndef COMMANDToFCUATT_H
#define COMMANDToFCUATT_H

#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>


#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/ControlCommand.h>

#include "pos_controller_DF.h"

class CommandToFcuAtt
{
    public:
        CommandToFcuAtt();
        ~CommandToFcuAtt();

        void init(ros::NodeHandle node);
        void run(double dt);

        void commandCallback(const prometheus_msgs::ControlCommand::ConstPtr &msg);
        void dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg);

    private:
        prometheus_msgs::DroneState drone_state_;
        prometheus_msgs::ControlCommand cmd_, cmd_last_;

        pos_controller_DF controller_;

        double takeoff_height_ ;      //默认起飞高度
        // double disarm_height_ ;      //到达该降落高度时，切换至land
        // double land_speed_;            //下降速度
        //bool has_takeoff_speed_;   //起飞标志，true表示采用用户指定上升速度。无效
        //double takeoff_speed_;        // 起飞速度
        bool landflag_;

        Eigen::Vector2d geo_fence_x_;    //地理围栏，超过该围栏。自动降落
        Eigen::Vector2d geo_fence_y_;
        Eigen::Vector2d geo_fence_z_;

        // mavros_msgs::PositionTarget pos_setpoint;
        Eigen::Vector3d pos_sp;     //期望的局部变量
        Eigen::Vector3d vel_sp;
        Eigen::Vector3d acc_sp;
        double yaw_sp;
        double yawdot_sp;

        ros::Publisher setpoint_raw_local_pub, setpoint_raw_attitude_pub;
;
        ros::Subscriber command_sub, drone_state_sub;

        ros::ServiceClient set_mode_client;//服务，改变PX4模式
        mavros_msgs::SetMode mode_cmd;
        ros::ServiceClient arming_client ;//服务，解锁和上锁PX4
        mavros_msgs::CommandBool arm_cmd;

        bool check_failsafe();
        void send_xyz_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp);
        void send_xyz_yaw_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, double yaw_sp);
        void send_att_setpoint(const double thrust_setpoint, const Eigen::Quaterniond attitude_setpoint);
};

#endif