/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

int goal_num_ =0;
int waypoint_num_;
double waypoints_[50][3];
mavros_msgs::State current_state;
Eigen::Vector3d odom_pos_;
bool odom_flag= true;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odom_cb(const nav_msgs::OdometryConstPtr& msg) 
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    double distance_to_goal = sqrt(pow(odom_pos_[0] - waypoints_[goal_num_-1][0], 2) + pow(odom_pos_[1] - waypoints_[goal_num_-1][1], 2));
         //+ pow(odom_pos_[2] - waypoints_[goal_num_-1][2], 2) );

    if(distance_to_goal<1.5&&goal_num_<waypoint_num_&&current_state.mode == "OFFBOARD")
    {
        odom_flag=true;
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);


    ros::Publisher goal_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10);


  nh.param("offboard_node/waypoint_num", waypoint_num_, -1);  
  for (int i = 0; i < waypoint_num_; i++)
  {
    nh.param("offboard_node/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("offboard_node/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("offboard_node/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>目标点自动分配<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< std::endl;

    // wait for FCU connection
    while(ros::ok() &&current_state.mode != "OFFBOARD" )
    {
        ros::spinOnce();
    }
    std::cout <<"共"<<waypoint_num_<<"个目标点"<<std::endl;
    std::cout <<"飞机已进入OFFBOARD模式，12s后发送目标点"<<std::endl;
    ros::Duration(12.0).sleep();

         
  while(ros::ok())
 {
     ros::spinOnce();

     if(odom_flag&&goal_num_<waypoint_num_)
     {
            nav_msgs::Path path;
            geometry_msgs::PoseStamped poses;
            poses.pose.position.x = waypoints_[goal_num_][0];
            poses.pose.position.y = waypoints_[goal_num_][1];
            poses.pose.position.z = waypoints_[goal_num_][2];

            path.poses.push_back(poses);

            goal_pub.publish(path);
            std::cout <<"第"<<goal_num_<<"个目标点已发送"<<std::endl;
            odom_flag=false;
            goal_num_++;
            if(goal_num_==waypoint_num_) 
            {
                std::cout <<"终点已发送!!"<<std::endl;
            }
     }

     ros::Duration(0.5).sleep();
 }
        
    

return 0;
}
