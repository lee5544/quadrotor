#include "Polygon.h"


Polygon::Polygon(){}

Polygon::~Polygon(){}

void Polygon::init(ros::NodeHandle node)
{
    ROS_INFO("run the polygon planner node");
    trigger_ = false;
    //【订阅】无人机当前状态
    drone_state_sub_ = node.subscribe("/prometheus/drone_state", 10, &Polygon::dronestateCallback, this);
    waypoint_sub_ = node.subscribe("/waypoint_generator/waypoints", 1, &Polygon::waypointCallback, this);
    traj_pub_ = node.advertise<prometheus_msgs::PositionReference>("/prometheus/polygon_planner/position_cmd", 50);
}

void Polygon::dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_state_ = *msg;
    cur_pos_ << drone_state_.position[0], drone_state_.position[1], drone_state_.position[2];
}

void Polygon::waypointCallback(const nav_msgs::PathConstPtr& msg) 
{
    std::cout << "Triggered!" << std::endl;
    trigger_ = true;
    goal_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;
    std::cout << "The goal is " << goal_.transpose() << std::endl;
}

void Polygon::run(int frequency)
{

    if(trigger_)
    {
        polygon_generator(cur_pos_, goal_, frequency, traj_);//生成一些列点
        trigger_ = false;
    }

    if(!traj_.empty())
    {
        std::cout <<traj_.back().transpose() << std::endl;
        polygon_planner_cmd_.header.stamp = ros::Time::now();
        polygon_planner_cmd_.header.frame_id = "map";

        polygon_planner_cmd_.move_mode = prometheus_msgs::PositionReference::xyz;  //TRAJECTORY
        polygon_planner_cmd_.move_frame = prometheus_msgs::PositionReference::ENU_FRAME; //ENU_FRAME

        polygon_planner_cmd_.position_ref[0] = traj_.back()[0];
        polygon_planner_cmd_.position_ref[1] = traj_.back()[1];
        polygon_planner_cmd_.position_ref[2] = traj_.back()[2];

        // polygon_planner_cmd_.velocity_ref[0] = 0;
        // polygon_planner_cmd_.velocity_ref[1] = 0;
        // polygon_planner_cmd_.velocity_ref[2] = 0;

        // polygon_planner_cmd_.acceleration_ref[0] = 0;
        // polygon_planner_cmd_.acceleration_ref[1] = 0;
        // polygon_planner_cmd_.acceleration_ref[2] = 0;

        // polygon_planner_cmd_.yaw_ref = yaw;
        traj_pub_.publish(polygon_planner_cmd_);

        traj_.pop_back();        
    }
}

void Polygon::polygon_generator(Eigen::Vector3d pos, Eigen::Vector3d goal, 
                                                                            int frequency, std::vector<Eigen::Vector3d> &traj)
{
    ROS_INFO("start generator ...");
    double vel = 1;
    Eigen::Vector3d delta;
    delta = goal - pos;

    int number = frequency * sqrt( delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2] ) / vel;
    
    for(int i = 0; i < number; i++)
    {
        traj.push_back(goal -  delta * i / number);
        // std::cout << (goal -  delta * i / number).transpose() << std::endl;
    }

    // std::cout << "the target numbers of  points: " << number << std::endl;
    std::cout << "the real numbers of  points: " << traj.size() << std::endl;

    if(traj.empty())
    {
        ROS_INFO("can not generate, failed to plan ...");
    }
}
