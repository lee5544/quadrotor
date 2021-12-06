#include "MissionRtk.h"


MissionRtk::MissionRtk(){}

MissionRtk::~MissionRtk(){}

void MissionRtk::fastplannerCallback(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    // planning_start = ros::Time::now().nsec; 
    // std::cout << "first obtain planning time: " << planning_start << std::endl;
    // std::cout << "the : " << planning_start - move_start << std::endl;

    fastplanner_cmd_ = *msg;
    hasPlanningPoints = true;
    // std::cout << fastplanner_cmd_.position_ref[0] << " " << fastplanner_cmd_.position_ref[1]
    //     << "    " << fastplanner_cmd_.position_ref[2] << std::endl;
}

void MissionRtk::dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_state_ = *msg;
}

void MissionRtk::homepositionCallback(const mavros_msgs::GPSRAW::ConstPtr& msg)
{
    if(isUpdateHome)//只在需要的时候更新
    { 
        isUpdateHome = false;

        home_position_.x() = drone_state_.position[0];//记录home位置
        home_position_.y() = drone_state_.position[1];
        home_position_.z() = drone_state_.position[2];
        hone_yaw_ = drone_state_.attitude[2];

        home_lon_ = double (msg->lon / 10000000.0);
        home_lat_ = double(msg->lat / 10000000.0);

        rtk2xy( home_lon_, home_lat_, homeUTME_, homeUTMN_);//计算经纬度的ENU

        double goalE, goalN;
        for(int  iter = 0; iter < all_goals_.size(); iter++)
        {
            rtk2xy(all_goals_[iter].longitude, all_goals_[iter].lantitude, goalE, goalN);//计算目标点经纬度的ENU

            all_goals_[iter].coordinate[0] = goalE - homeUTME_ + home_position_[0];
            all_goals_[iter].coordinate[1] = goalN - homeUTMN_ + home_position_[1];
            all_goals_[iter].coordinate[2] = takeoff_height_ + home_position_[2];
            // std::cout << iter << "  "<<all_goals_[iter].point.transpose() << std::endl;

        }
        final_goal_ = all_goals_[all_goals_.size()-1];

        readyTotakeoff = true;

        std::cout << "      the home position(x,y,z,yaw): " << home_position_.transpose() << ", " << hone_yaw_*53.7<< std::endl;
        std::cout << "      the home position(lon,lat): " << home_lon_ << ", " <<home_lat_<<std::endl;
        std::cout << "      the home position(E,N): " << homeUTME_ << ", " <<homeUTMN_<<std::endl;
    }

    // 测试RTK数据以及转换后的精度，小数点后第5位约表示1m
    // double lon = double (msg->lon / 10000000.0);
    // double lat = double(msg->lat / 10000000.0);
    // std::cout << "the position(lon,lat): " << lon << ", " <<lat;
    // double tempE, tempN;
    // rtk2xy( lon, lat, tempE, tempN);//计算经纬度的ENU
    // std::cout << "      (E,N): " << tempE << ", " <<tempN<<std::endl;
}

void MissionRtk::mr72ArmingCallback(const can_rec::trigger::ConstPtr& msg)
{
    if(msg->flag == 1)//预警，降低速度
    {
        isMr72SlowDown = true;
    }else
    {
        isMr72SlowDown = false;
    }

    nav_msgs::Path path;
    geometry_msgs::PoseStamped poses;
    poses.pose.position.x = goal_.coordinate[0];
    poses.pose.position.y = goal_.coordinate[1];
    poses.pose.position.z = goal_.coordinate[2];
    path.poses.push_back(poses);

    poses.pose.position.x = goal_.vel;//最大速度
    poses.pose.position.y = goal_.acc;//最大加速度
    poses.pose.position.z = 0;//最大速度
    if( isMr72SlowDown)
    {
        poses.pose.position.x = 1.0;//最大速度
        poses.pose.position.y = 1.5;//最大加速度
        poses.pose.position.z = 0;//最大速度              
    }    
    path.poses.push_back(poses);
    
    if(useMr72 && all_goals_.size() > 0)
    {
        goal_pub.publish(path);            
    }
}

void MissionRtk::init(ros::NodeHandle node)
{
    //【订阅】无人机当前状态
    drone_state_sub = node.subscribe("/prometheus/drone_state", 10, &MissionRtk::dronestateCallback, this);
    home_postion_sub   =   node.subscribe("/radar/alarming", 10, &MissionRtk::mr72ArmingCallback, this);    
    
    //【订阅】
    fastplanner_sub   =   node.subscribe("/prometheus/fast_planner/position_cmd", 50, &MissionRtk::fastplannerCallback, this);
    mr72Arming_sub = node.subscribe("/mavros/gpsstatus/gps1/raw", 10, &MissionRtk::homepositionCallback, this);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = node.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, this);
    goal_pub = node.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10, this);


    takeoff_height_ = 1.5;
    hover_duration_ = 3;//悬停时间 3s
    chose_planner_ = planner_::kino;//规划方法
    // chose_planner_ = planner_::no_planner;//规划方法

    // node.param("point_num", all_goals_size_, -1);
    //  设置目标点
    //  位置参数，最大速度，最大加速度
    // Point point3;
    //     point3.longitude = 0; point3.lantitude = 0; point3.height = takeoff_height_; 
    //     point3.vel=2; point3.acc=2;    
    //     all_goals_.push_back(point3);
    // Point point2;
    //     point2.longitude = 0; point2.lantitude = 0; point2.height = takeoff_height_; 
    //     point2.vel=2; point2.acc=2;    
    //     all_goals_.push_back(point2);
    // Point point1;
    //     point1.longitude = 0; point1.lantitude = 0; point1.height = takeoff_height_; 
    //     point1.vel=1; point1.acc=2;
    //     all_goals_.push_back(point1);    
    Point point0;//(10,0,1.5)
        point0.longitude = 8.5457232; point0.lantitude = 47.3977424; point0.height = takeoff_height_; 
        point0.vel=1; point0.acc=1.5;
        all_goals_.push_back(point0);    

    all_goals_size_ = all_goals_.size();
    // std::cout << all_goals_size_ << std::endl;

    mission_cmd_ = mission_fsm_::idle;
    isTakeoff = false; readyTotakeoff = false; 
    isTakeoff2Hold = false; isMove2Hold = false;
    isMove = false; isUpdateHome = false;
    isSendGoal = true;
    hasPlanningPoints = false;

    useMr72 = true;

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    command_now_.mode                                = prometheus_msgs::ControlCommand::idle;
    command_now_.source = "NODE_NAME";
    command_now_.reference_state.move_mode         = prometheus_msgs::PositionReference::xyz_yaw;
    command_now_.reference_state.move_frame         = prometheus_msgs::PositionReference::ENU_FRAME;
    command_now_.reference_state.position_ref[0]    = 0;
    command_now_.reference_state.position_ref[1]    = 0;
    command_now_.reference_state.position_ref[2]    = 0;
    command_now_.reference_state.velocity_ref[0]     = 0;
    command_now_.reference_state.velocity_ref[1]     = 0;
    command_now_.reference_state.velocity_ref[2]     = 0;
    command_now_.reference_state.acceleration_ref[0] = 0;
    command_now_.reference_state.acceleration_ref[1] = 0;
    command_now_.reference_state.acceleration_ref[2] = 0;
    command_now_.reference_state.yaw_ref             = 0;
    
    ROS_INFO("initalization");
    std::cout<<"    takeoff height: " << takeoff_height_ <<std::endl;
    std::cout<<"    hold time:(/s) " << hover_duration_ <<std::endl;
    std::cout<<"    goals size " << all_goals_size_ <<std::endl;
    if(chose_planner_ == planner_::kino){
        std::cout<<"    planning method: " << "planner_::kino" <<std::endl;
    }
    else if(chose_planner_ == planner_::no_planner){
        std::cout<<"    planning method: " << "planner_::no_planner" <<std::endl;
    }
    if(useMr72){
        std::cout<<"    use MR72 to reduce the speed"<<std::endl;
    }
}

/**
 * @Author:       yong
 * @description:         mission状态机函数，负责任务分解和组合
 * @param {*}
 * @return {*}
 */
void MissionRtk::run()
{
        // if(){    //  错误保护机制
        //     return 0;
        // }
        switch(mission_cmd_){
            case mission_fsm_::idle:{
                // if  (drone_state_.mode == "OFFBOARD")//解锁后，进入takeoff，指明起飞高度，记录home位置
                if  (drone_state_.armed)//解锁后，进入takeoff，指明起飞高度，记录home位置
                {
                    ROS_INFO("The mission is started ......");
                    ROS_INFO("IDLE to TAKEOFF");
                    mission_cmd_ = mission_fsm_::takeoff;   isTakeoff = true;
                }
                else
                {
                    mission_cmd_ = mission_fsm_::idle;
                    command_now_.header.stamp = ros::Time::now();
                    command_now_.mode = prometheus_msgs::ControlCommand::idle;
                    command_pub.publish(command_now_);
                }
                
                break;
            }

            case mission_fsm_::takeoff:{
                if(isTakeoff){
                    isTakeoff = false;
                    isUpdateHome = true;// 发送更新home_gps的命令          
                }
                
                if(readyTotakeoff){
                    readyTotakeoff = false;
                    command_now_.mode = prometheus_msgs::ControlCommand::takeoff;   
                    command_now_.reference_state.position_ref[0] = home_position_[0];
                    command_now_.reference_state.position_ref[1] = home_position_[1];
                    command_now_.reference_state.position_ref[2] = takeoff_height_ + home_position_[2];//当高度大于0.5m时，有效
                    command_now_.reference_state.yaw_ref = hone_yaw_;
                    command_now_.header.stamp = ros::Time::now();    
                    
                    std::cout << "      the takeoff target is " <<  home_position_[0] << "    " << home_position_[1] << "     " 
                                       << takeoff_height_ + home_position_[2] << std::endl;
                }
                else if (command_now_.reference_state.position_ref[2] > 0.5 && 
                                    drone_state_.position[2] > command_now_.reference_state.position_ref[2]-0.2)//到达指定高度，进入hold
                {
                    ROS_INFO("TAKEOFF to HOLD");
                    mission_cmd_ = mission_fsm_::hold;    isTakeoff2Hold = true;
                    command_now_.header.stamp = ros::Time::now();
                    command_now_.mode = prometheus_msgs::ControlCommand::hold;
                }
                command_pub.publish(command_now_); 
                
                break;
            }
            
            case mission_fsm_::hold:{
                if(isTakeoff2Hold){
                    isTakeoff2Hold = false;
                    holdtime_start = ros::Time::now().sec;
                    
                    command_now_.mode = prometheus_msgs::ControlCommand::hold;   
                    command_now_.reference_state.position_ref[0] = home_position_[0];
                    command_now_.reference_state.position_ref[1] = home_position_[1];
                    command_now_.reference_state.position_ref[2] = takeoff_height_ + home_position_[2];
                    command_now_.reference_state.yaw_ref = hone_yaw_;

                    std::cout << "      takeoff to hold, the hold target is " <<  home_position_[0] << ", " << home_position_[1] << ", " 
                                       << takeoff_height_ + home_position_[2] << ", " << hone_yaw_*57.3<< std::endl;                    
                }
                if(isMove2Hold)
                {
                    isMove2Hold = false;
                    holdtime_start = ros::Time::now().sec;
                    
                    command_now_.mode = prometheus_msgs::ControlCommand::hold;   
                    command_now_.reference_state.position_ref[0] = final_goal_.coordinate[0];
                    command_now_.reference_state.position_ref[1] = final_goal_.coordinate[1];
                    command_now_.reference_state.position_ref[2] = final_goal_.coordinate[2];
                    // command_now_.reference_state.yaw_ref = hone_yaw_;

                    std::cout << "      move to hold, the hold target is " <<  final_goal_.coordinate.transpose() << std::endl;          
                }

                holdtime = ros::Time::now().sec;

                if (all_goals_.empty())//无目标点，悬停xs，降落
                {
                    if((holdtime - holdtime_start) > hover_duration_){
                        ROS_INFO("HOLD to LAND, no goal");
                        ROS_INFO("The mission is over ......");
                        mission_cmd_ = mission_fsm_::land;
                        command_now_.header.stamp = ros::Time::now();
                        command_now_.mode = prometheus_msgs::ControlCommand::land;
                    }
                    else{
                        mission_cmd_ = mission_fsm_::hold;
                        command_now_.header.stamp = ros::Time::now();
                        command_now_.mode = prometheus_msgs::ControlCommand::hold;    
                    }
                }
                else if(!all_goals_.empty() && (holdtime - holdtime_start) > hover_duration_){//有目标点，悬停时间，进入move
                    ROS_INFO("HOLD to MOVE");
                    mission_cmd_ = mission_fsm_::move;    isMove = true;
                    command_now_.header.stamp = ros::Time::now();
                    command_now_.mode = prometheus_msgs::ControlCommand::move;                                    
                }
                else{//悬停状态
                    command_now_.header.stamp = ros::Time::now();
                    command_now_.mode = prometheus_msgs::ControlCommand::hold;
                }
                command_pub.publish(command_now_);
                
                break;
            }

            case mission_fsm_::move:{
                if(isMove)//hold进入move，目标点加上（减去）起点位置
                {
                    isMove = false;
                    std::cout << "      there are " << all_goals_.size() << " goals" << std::endl;
                }

                if (!all_goals_.empty()){
                    //发送目标点
                    if(all_goals_.size() > 0 && isSendGoal)//防止vector尾部超出的部分
                    {
                        isSendGoal = false;
                        ROS_INFO("MOVE, send goal");
                        goal_ = all_goals_.back();//获取目标集最新的点

                        nav_msgs::Path path;
                        geometry_msgs::PoseStamped poses;
                        poses.pose.position.x = goal_.coordinate[0];
                        poses.pose.position.y = goal_.coordinate[1];
                        poses.pose.position.z = goal_.coordinate[2];
                        path.poses.push_back(poses);

                        poses.pose.position.x = goal_.vel;//最大速度
                        poses.pose.position.y = goal_.acc;//最大加速度
                        poses.pose.position.z = 0;//最大速度

                        path.poses.push_back(poses);
                        goal_pub.publish(path);    
                        std::cout <<"       the "<< all_goals_size_  - all_goals_.size() << "th goal: " << goal_.coordinate.transpose() 
                                        <<  "   The max vel and acc are:  " << goal_.vel<< " " << goal_.acc<<std::endl;                 
                    }            
                    
                    //没有避障模块
                    if(chose_planner_ == planner_::no_planner)
                    {
                        command_now_.header.stamp = ros::Time::now();
                        command_now_.mode                                = prometheus_msgs::ControlCommand::move;
                        command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                        // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                        // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_vel_yaw;
                        command_now_.reference_state.position_ref =  {goal_.coordinate[0], goal_.coordinate[1], goal_.coordinate[2]};

                        command_now_.reference_state.yaw_ref = drone_state_.attitude[2];                    
                    }
                    
                    //避障模块工作
                    if(chose_planner_ == planner_::kino)
                    {
                        if (hasPlanningPoints)//当规划轨迹来了
                        {
                            hasPlanningPoints = false;
                            command_now_.reference_state =  fastplanner_cmd_;//默认xyz_vel_yaw
                            command_now_.header.stamp = ros::Time::now();
                            command_now_.mode = prometheus_msgs::ControlCommand::move;
                            // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                            // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                            command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_vel_yaw;
                            // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::trajectory;
                            command_now_.reference_state.yaw_ref = fastplanner_cmd_.yaw_ref;
                        }
                        else//悬停在当前位置
                        {
                            std::cout << "      no planning trajectory" << std::endl;
                            // command_now_.header.stamp = ros::Time::now();
                            // command_now_.mode = prometheus_msgs::ControlCommand::move;
                            // // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_yaw;
                            // // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::vel_yaw;
                            // command_now_.reference_state.move_mode  = prometheus_msgs::PositionReference::xyz_vel_yaw;
                            // command_now_.reference_state.position_ref =  drone_state_.position;
                            // command_now_.reference_state.yaw_ref = fastplanner_cmd_.yaw_ref;
                        }
                    }

                    //判断是否到达目标点，并且弹出
                    if( sqrt(pow(drone_state_.position[0] - goal_.coordinate[0], 2) + 
                                            pow(drone_state_.position[1] - goal_.coordinate[1], 2)) < 0.5)
                    {
                        all_goals_.pop_back();
                        isSendGoal = true;
                    }
                
                }
                else{
                    ROS_INFO("MOVE to HOLD, no goal");
                    mission_cmd_ = mission_fsm_::hold;    isMove2Hold = true;
                    command_now_.header.stamp = ros::Time::now();
                    command_now_.mode = prometheus_msgs::ControlCommand::hold;
                }

                command_pub.publish(command_now_);
                break;
            }

            case mission_fsm_::land:{
                mission_cmd_ = mission_fsm_::land;
                command_now_.header.stamp = ros::Time::now();
                command_now_.mode = prometheus_msgs::ControlCommand::land;
                // if(!drone_state_.armed)
                // {
                //     mission_cmd_ = mission_fsm_::idle;
                //     command_now_.header.stamp = ros::Time::now();
                //     command_now_.mode = prometheus_msgs::ControlCommand::idle;
                //     ROS_INFO("LAND to IDLE");
                // }
                
                command_pub.publish(command_now_);             
                break;
            }
        }
        // last_mission_cmd_ = mission_cmd_;
}

/**
 * @Author:       yong
 * @description:         计算gps坐标的UTM坐标
 * @param 
 *      lon：经度   lat：纬度   UTME：东向坐标  UTMN：北向坐标
 * @return {*}
 */
void MissionRtk::rtk2xy(double lon, double lat, double& UTME, double& UTMN)
{
    // ROS_INFO("gps to xyz");
	double kD2R = 3.1415926 / 180.0;
	double ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
	double L0 = ZoneNumber * 3.0;

	double a = 6378137.0;
	double F = 298.257223563;
	double f = 1 / F;
	double b = a * (1 - f);
	double ee = (a * a - b * b) / (a * a);
	double e2 = (a * a - b * b) / (b * b);
	double n = (a - b) / (a + b); 
	double n2 = (n * n); 
	double n3 = (n2 * n); 
	double n4 = (n2 * n2); 
	double n5 = (n4 * n);
	double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
	double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
	double gm = 15 * n2 / 16 - 15 * n4 / 32;
	double dt = -35 * n3 / 48 + 105 * n5 / 256;
	double ep = 315 * n4 / 512;
	double B = lat * kD2R;
	double L = lon * kD2R;
	L0 = L0 * kD2R;
	double l = L - L0; 
	double cl = (cos(B) * l); 
	double cl2 = (cl * cl); 
	double cl3 = (cl2 * cl); 
	double cl4 = (cl2 * cl2); 
	double cl5 = (cl4 * cl); 
	double cl6 = (cl5 * cl); 
	double cl7 = (cl6 * cl); 
	double cl8 = (cl4 * cl4);
	double lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
	double t = tan(B); 
	double t2 = (t * t); 
	double t4 = (t2 * t2); 
	double t6 = (t4 * t2);
	double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
	double yt = e2 * cos(B) * cos(B);
	double N = lB;
	N = N + t * Nn * cl2 / 2;
	N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
	N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
	N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
	double E = Nn * cl;
	E = E + Nn * cl3 * (1 - t2 + yt) / 6;
	E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
	E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
	E = E + 500000;
	N = 0.9996 * N;
	E = 0.9996 * (E - 500000.0) + 500000.0;

	UTME = E;
	UTMN = N;
    // std::cout << UTME << "  " << UTMN << std::endl;
}
