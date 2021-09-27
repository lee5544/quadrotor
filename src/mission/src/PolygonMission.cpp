#include "PolygonMission.h"


PolygonMission::PolygonMission(){}

PolygonMission::~PolygonMission(){}

void PolygonMission::polygonplannerCallback(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    // planning_start = ros::Time::now().nsec; 
    // std::cout << "first obtain planning time: " << planning_start << std::endl;
    // std::cout << "the : " << planning_start - move_start << std::endl;

    polygon_planner_cmd_ = *msg;
    hasPlanningPoints_ = true;
}

void PolygonMission::dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_state_ = *msg;
}

void PolygonMission::init(ros::NodeHandle node)
{

    //【订阅】无人机当前状态
    drone_state_sub = node.subscribe("/prometheus/drone_state", 10, &PolygonMission::dronestateCallback, this);
    //【订阅】来自planning的指令
    polygon_planner_sub   =   node.subscribe("/prometheus/polygon_planner/position_cmd", 50, &PolygonMission::polygonplannerCallback, this);
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = node.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, this);

    goal_pub = node.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10, this);

    //  设置目标点
    Eigen::Vector3d point3; point3 << 0,0,1.5;  goalpoints_.push_back(point3);
    Eigen::Vector3d point2; point2 << 0,10,1.5;  goalpoints_.push_back(point2);
    Eigen::Vector3d point1; point1 << 10,10,1.5;  goalpoints_.push_back(point1);
    Eigen::Vector3d point0; point0 << 10,0,1.5;  goalpoints_.push_back(point0);
    
    // Eigen::Vector3d point0; point0 << 0,10,2;  goalpoints_.push_back(point0);

    goalsize_ = goalpoints_.size();
    // std::cout << goalsize_ << std::endl;

    // chose_planner_ = planner_::kino;//规划方法
    // chose_planner_ = planner_::no_planner;//规划方法
    chose_planner_ = planner_::polygon;//规划方法

    takeoff_height_ = 1.5;
    hover_duration_ = 5;//悬停时间 5s
    hover_max_ = 15;//最大悬停时间15s
    mission_cmd_ = mission_fsm_::idle;

    ishold = false; ismove = false;
    isSendGoal = true;
    hasPlanningPoints_ = false;

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.mode                                = prometheus_msgs::ControlCommand::idle;
    Command_Now.source = "NODE_NAME";
    Command_Now.reference_state.move_mode           = prometheus_msgs::PositionReference::xyz_yaw;
    Command_Now.reference_state.move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.reference_state.position_ref[0]     = 0;
    Command_Now.reference_state.position_ref[1]     = 0;
    Command_Now.reference_state.position_ref[2]     = 0;
    Command_Now.reference_state.velocity_ref[0]     = 0;
    Command_Now.reference_state.velocity_ref[1]     = 0;
    Command_Now.reference_state.velocity_ref[2]     = 0;
    Command_Now.reference_state.acceleration_ref[0] = 0;
    Command_Now.reference_state.acceleration_ref[1] = 0;
    Command_Now.reference_state.acceleration_ref[2] = 0;
    Command_Now.reference_state.yaw_ref             = 0;
}

void PolygonMission::run()
{
        switch(mission_cmd_){
            case mission_fsm_::idle:
                if  (drone_state_.armed)//解锁后，进入takeoff，指明起飞高度，记录home位置
                {
                    ROS_INFO("IDLE to TAKEOFF");
                    
                    home_position_.x() = drone_state_.position[0];
                    home_position_.y() = drone_state_.position[1];
                    home_position_.z() = drone_state_.position[2];

                    mission_cmd_ = mission_fsm_::takeoff;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::takeoff;
                    Command_Now.reference_state.position_ref[2]     = takeoff_height_ + drone_state_.position[2];//当高度大于0.5m时，有效
                }
                else
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::idle;
                }
                command_pub.publish(Command_Now);
            break;

            case mission_fsm_::takeoff:
                if (drone_state_.position[2] > Command_Now.reference_state.position_ref[2]-0.1)//到达指定高度，进入hold
                {
                    ROS_INFO("TAKEOFF to HOLD");
                    mission_cmd_ = mission_fsm_::hold;    ishold = true;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::hold;
                }
                else
                {
                    mission_cmd_ = mission_fsm_::takeoff;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::takeoff;        
                }
                command_pub.publish(Command_Now);    
            break;

            case mission_fsm_::hold:
                if(ishold){
                    holdtime_start = ros::Time::now().sec;
                    ishold = false;
                }
                holdtime = ros::Time::now().sec;

                if((holdtime - holdtime_start) > hover_max_)//超过最大悬停时间，降落
                {
                    ROS_INFO("HOLD to LAND, over 15s");
                    mission_cmd_ = mission_fsm_::land;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::land;
                }
                else if (goalpoints_.empty())//无目标点，悬停5s，降落
                {        
                    if((holdtime - holdtime_start) > hover_duration_){
                        ROS_INFO("HOLD to LAND, no goal");
                        ROS_INFO("The mission is over ......");
                        mission_cmd_ = mission_fsm_::land;
                        Command_Now.header.stamp = ros::Time::now();
                        Command_Now.mode = prometheus_msgs::ControlCommand::land;
                    }
                    else{
                        mission_cmd_ = mission_fsm_::hold;
                        Command_Now.header.stamp = ros::Time::now();
                        Command_Now.mode = prometheus_msgs::ControlCommand::hold;    
                    }
                }
                else if (!goalpoints_.empty() && (holdtime - holdtime_start) > hover_duration_)//有目标点，悬停时间大于5s，进入move
                {
                    ROS_INFO("HOLD to MOVE");
                    mission_cmd_ = mission_fsm_::move;    ismove = true;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::move;
                }
                else{
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::hold;                    
                }
                
                command_pub.publish(Command_Now);
            break;

            case mission_fsm_::move:
                if(ismove == true)//第一次进入move，目标点加上（减去）起点位置
                {
                    // move_start = ros::Time::now().nsec; std::cout << "move start time : " << move_start << std::endl;
                    //对目标点加上Home位置（局部XYZ）——相对
                    //对目标点减去Home位置（全局GPS），算出相对的目标xyz，再加上local的局部xyz
                    for(int  iter = 0; iter < goalpoints_.size(); iter++)
                    {
                        goalpoints_[iter].x() += home_position_.x();
                        goalpoints_[iter].y() += home_position_.y();
                        goalpoints_[iter].z() = drone_state_.position[2];
                        // std::cout << iter << "  "<<goalpoints_[iter].transpose() << std::endl;
                    }
                    // goal_ = goalpoints_.back();//获取目标集最新的点

                    std::cout << "There are " << goalpoints_.size() << " goals" << std::endl;
                    ismove = false;
                }

                if (!goalpoints_.empty()){
                    //发送目标点
                    if(goalpoints_.size() > 0 && isSendGoal)//防止vector尾部超出的部分
                    {
                        isSendGoal = false;
                        ROS_INFO("MOVE, send goal");
                        goal_ = goalpoints_.back();//获取目标集最新的点

                        nav_msgs::Path path;
                        geometry_msgs::PoseStamped poses;
                        poses.pose.position.x = goal_[0];
                        poses.pose.position.y = goal_[1];
                        poses.pose.position.z = goal_[2];
                        path.poses.push_back(poses);
                        goal_pub.publish(path);    

                        std::cout <<"The "<< goalsize_  - goalpoints_.size() << "th goal: " << goal_.transpose() << std::endl;                      
                    }            
                    
                    if(chose_planner_ == planner_::no_planner)
                    {
                        Command_Now.header.stamp = ros::Time::now();
                        Command_Now.mode                                = prometheus_msgs::ControlCommand::move;
                        Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                        // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                        // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_vel_yaw;
                        Command_Now.reference_state.position_ref =  {goal_[0], goal_[1], goal_[2]};
                        Command_Now.reference_state.yaw_ref = drone_state_.attitude[2];                    
                    }

                    //多边形规划模块工作
                    if(chose_planner_ == planner_::polygon)
                    {
                        if (hasPlanningPoints_)//当规划轨迹来了
                        {
                            // std::cout << "send a target!" << std::endl;
                            hasPlanningPoints_ = false;
                            Command_Now.header.stamp = ros::Time::now();
                            Command_Now.mode = prometheus_msgs::ControlCommand::move;
                            // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                            // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                            Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz;
                            Command_Now.reference_state =  polygon_planner_cmd_;
                            // Command_Now.reference_state.yaw_ref = polygon_planner_cmd_.yaw_ref;
                        }
                        // else//悬停在当前位置
                        // {
                        //     std::cout << "no planning trajectory" << std::endl;
                        //     Command_Now.header.stamp = ros::Time::now();
                        //     Command_Now.mode = prometheus_msgs::ControlCommand::move;
                        //     Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                        //     // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                        //     // Command_Now.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_vel_yaw;
                        //     Command_Now.reference_state.position_ref =  drone_state_.position;
                        //     // Command_Now.reference_state.yaw_ref = polygon_planner_cmd_.yaw_ref;
                        // }
                    }

                    //判断是否到达目标点，并且弹出
                    if( sqrt(pow(drone_state_.position[0] - goal_[0], 2) + 
                                            pow(drone_state_.position[1] - goal_[1], 2)) < 0.5)
                    {
                        goalpoints_.pop_back();
                        isSendGoal = true;
                    }
                }
                else{
                    ROS_INFO("MOVE to HOLD, no goal");
                    mission_cmd_ = mission_fsm_::hold;    ishold = true;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.mode = prometheus_msgs::ControlCommand::hold;
                }

                command_pub.publish(Command_Now);
            break;

            case mission_fsm_::land:
                mission_cmd_ = mission_fsm_::land;
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.mode = prometheus_msgs::ControlCommand::land;
                // if(!drone_state_.armed)
                // {
                //     mission_cmd_ = mission_fsm_::idle;
                //     Command_Now.header.stamp = ros::Time::now();
                //     Command_Now.mode = prometheus_msgs::ControlCommand::idle;
                //     ROS_INFO("LAND to IDLE");
                // }
                
                command_pub.publish(Command_Now);             
            break;
        }
}