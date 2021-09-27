#include "CommandToFcu.h"

CommandToFcu::CommandToFcu()
{}
CommandToFcu::~CommandToFcu()
{}

void CommandToFcu::init(ros::NodeHandle node)
{
    ROS_INFO("send commands to fcu!");
    takeoff_height_ = 1.0;                                       //默认起飞高度
    // disarm_height_ = 0.3;
    // land_speed_ = 0.2;
    //has_takeoff_speed_ = false;
    //takeoff_speed_ = 0.3;
    landflag_ = true;

    geo_fence_x_[0] = -100; geo_fence_x_[1] = 100;
    geo_fence_y_[0] = -100; geo_fence_y_[1] = 100;
    geo_fence_z_[0] = -100;geo_fence_z_[1] = 100;

    cmd_.mode = 0;
    
    setpoint_raw_local_pub = node.advertise<mavros_msgs::PositionTarget>( "/mavros/setpoint_raw/local", 10);
    set_mode_client = node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    drone_state_sub = node.subscribe("/prometheus/drone_state", 10, &CommandToFcu::dronestateCallback, this);
    command_sub = node.subscribe("/prometheus/control_command", 100, &CommandToFcu::commandCallback, this);
}

void CommandToFcu::commandCallback(const prometheus_msgs::ControlCommand::ConstPtr &msg)
{
    cmd_ = *msg;
    // run(0.02);//数值为运行频率的倒数
}

void CommandToFcu::dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_state_ = *msg;
}

bool CommandToFcu::check_failsafe()
{
    if (drone_state_.position[0] < geo_fence_x_[0] || drone_state_.position[0] > geo_fence_x_[1] ||
        drone_state_.position[1] < geo_fence_y_[0] || drone_state_.position[1] > geo_fence_y_[1] ||
        drone_state_.position[2] < geo_fence_z_[0] || drone_state_.position[2] > geo_fence_z_[1])
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CommandToFcu::run(double dt)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // Check for geo fence: If drone is out of the geo fence, it will land now.
    if(check_failsafe() == true)//地理围栏检测
    {
        cmd_.mode = prometheus_msgs::ControlCommand::land;
    }
        
    switch (cmd_.mode)
    {
        case prometheus_msgs::ControlCommand::idle:
        {
            // ROS_INFO("idle!");
                if(cmd_last_.mode != prometheus_msgs::ControlCommand::idle)
                {
                    ROS_INFO("Wait for ARMMING and OFFBOARD!");
                }
                pos_setpoint.type_mask = 0x4000;
                pos_setpoint.coordinate_frame = 1;
                setpoint_raw_local_pub.publish(pos_setpoint);

                // pos_sp = Eigen::Vector3d(drone_state_.position[0], drone_state_.position[1], drone_state_.position[2]);
                // yaw_sp = drone_state_.local_euler[2];//当前航向角
                // send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);   
                
                // if(cmd_.yaw_ == 999)//自动解锁，进入offboard
                // {
                //     // if (cmd_last_.mode != idle)
                //     //     ROS_INFO("--------------------------------------------------idle-----------------------------------------------------");
                //     if(!drone_state_.armed)
                //     {
                //         arm_cmd.request.value = true;
                //         if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                //         {
                //             ROS_INFO("Vehicle armed "); 
                //         }
                //     }                        
                //     if(drone_state_.mode != "OFFBOARD")
                //     {
                //         mode_cmd.request.custom_mode = "OFFBOARD";
                //         set_mode_client.call(mode_cmd);
                //         if(drone_state_.mode == "OFFBOARD")
                //         {   
                //             ROS_INFO("OFFBOARD enabled ");   
                //         }                          
                //     }
                // }
            break;            
        }

        case prometheus_msgs::ControlCommand::takeoff:
        {
            // ROS_INFO("takeoff!");
                if (cmd_last_.mode != prometheus_msgs::ControlCommand::takeoff)//防止多次起飞
                {
                    ROS_INFO("The drone is going to takeoff!");
                        
                    pos_sp = Eigen::Vector3d(drone_state_.position[0], drone_state_.position[1], drone_state_.position[2] + takeoff_height_);
                    yaw_sp = drone_state_.attitude[2];//当前航向角
                    if(cmd_.reference_state.position_ref[2] > 0.5)//判断用户输入起飞高度是否有效
                    {
                        pos_sp[2] = cmd_.reference_state.position_ref[2];
                    }
                    // cmd_.reference_state.position_ref = {pos_sp[0], pos_sp[1], pos_sp[1]};
                    // cmd_.reference_state.yaw_ref = yaw_sp;
                    std::cout<<"The takeoff target is :     "<< pos_sp.transpose() << "   " << yaw_sp*53.7 << std::endl;
                }
                send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);  

            break;            
        }

        case prometheus_msgs::ControlCommand::hold:
        {
            // ROS_INFO("hold!");
                if (cmd_last_.mode != prometheus_msgs::ControlCommand::hold)//仅仅记录一次当前位置
                {
                    //记录位置
                    ROS_INFO("The drone is hovering!");
                    pos_sp = Eigen::Vector3d(drone_state_.position[0], drone_state_.position[1], drone_state_.position[2]);
                    yaw_sp = drone_state_.attitude[2];//当前航向角
                    // cmd_.reference_state.position_ref = {pos_sp[0], pos_sp[1], pos_sp[1]};
                    // cmd_.reference_state.yaw_ref = yaw_sp; 
                    std::cout<<"The hold target is :     "<< pos_sp.transpose() << "   " << yaw_sp*53.7 << std::endl;                                       
                }
                send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);
            break;            
        }

        case prometheus_msgs::ControlCommand::move:
        {
             // ROS_INFO("move!");
            if (cmd_last_.mode != prometheus_msgs::ControlCommand::move)//仅仅记录一次当前位置
                ROS_INFO("The drone is moving!");
            //  获取目标点
            //  根据模式，选择相应的控制模式
            if( cmd_.reference_state.move_mode  == prometheus_msgs::PositionReference::xyz )
            {
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                send_xyz_setpoint(pos_setpoint, pos_sp);
            }
            if( cmd_.reference_state.move_mode  == prometheus_msgs::PositionReference::xyz_yaw )//已经测试
            {
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                yaw_sp = cmd_.reference_state.yaw_ref;
                send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);
            }
            else if( cmd_.reference_state.move_mode  == prometheus_msgs::PositionReference::xyz_vel_yaw )//已经测试
            {
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                vel_sp << cmd_.reference_state.velocity_ref[0], cmd_.reference_state.velocity_ref[1], cmd_.reference_state.velocity_ref[2];
                yaw_sp = cmd_.reference_state.yaw_ref;
                
                send_xyz_vel_yaw_setpoint(pos_setpoint, pos_sp, vel_sp, yaw_sp);
            }
            else if( cmd_.reference_state.move_mode == prometheus_msgs::PositionReference::vel_yaw)//已经测试
            {
                vel_sp << cmd_.reference_state.velocity_ref[0], cmd_.reference_state.velocity_ref[1], cmd_.reference_state.velocity_ref[2];
                yaw_sp = cmd_.reference_state.yaw_ref;
                send_vel_yaw_setpoint(pos_setpoint, vel_sp, yaw_sp);
            }
            // else if( cmd_.move_mode  == xyz_vel_yaw_yawdot )
            // {
            //     pos_sp = cmd_.pos_;
            //     vel_sp = cmd_.vel_;
            //     yaw_sp = cmd_.yaw_;
            //     yawdot_sp = cmd_.yaw_rate_;
            //     send_xyz_vel_yaw_yawdot_setpoint(pos_setpoint, pos_sp, vel_sp, yaw_sp, yawdot_sp);
            // }
            // else if( cmd_.move_mode  == xyz_vel_acc_yaw_yawdot )
            // {
            //     pos_sp = cmd_.pos_;
            //     vel_sp = cmd_.vel_;
            //     acc_sp = cmd_.acc_;
            //     yaw_sp = cmd_.yaw_;
            //     yawdot_sp = cmd_.yaw_rate_;
            //     send_xyz_vel_acc_yaw_yawdot_setpoint(pos_setpoint, pos_sp, vel_sp, acc_sp, yaw_sp, yawdot_sp);
            // }
            break;           
        }

        case prometheus_msgs::ControlCommand::land:
        {
            if (cmd_last_.mode != prometheus_msgs::ControlCommand::land)//记录当前位置，只进入一次
            {
                pos_sp[0] = drone_state_.position[0];
                pos_sp[0] = drone_state_.position[1];
                yaw_sp = drone_state_.attitude[2];//当前航向角
                // cmd_.pos_ = drone_state_.position;
                // cmd_.pos_[2] = 0;
                // cmd_.yaw_ = drone_state_.local_euler[2];
                ROS_INFO("The drone is landing!");
            }
            // if(drone_state_.position[2] > disarm_height_)
            // {
            //     pos_sp[2] = drone_state_.position[2] - land_speed_ * dt;
            //     vel_sp[0] = 0;
            //     vel_sp[1] = 0;
            //     vel_sp[2] = -land_speed_;
            //     send_xyz_vel_yaw_setpoint(pos_setpoint, pos_sp, vel_sp, yaw_sp);
            // }
            // else if(drone_state_.position[2] <= disarm_height_)//退出offboard模式
            // {
            //     if(drone_state_.mode != "AUTO.LAND") 
            //     {
            //         //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
            //         mode_cmd.request.custom_mode = "AUTO.LAND";
            //         set_mode_client.call(mode_cmd);
            //         if(drone_state_.mode == "AUTO.LAND")
            //         {   ROS_INFO("AUTO.LAND enabled");   }  
            //     }
            // }

            if(drone_state_.mode != "AUTO.LAND")//利用服务-客户机制，改变PX4模式至land
            {
                //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
                mode_cmd.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(mode_cmd);

                // if( set_mode_client.call(mode_cmd) &&mode_cmd.response.mode_sent){
                //     ROS_INFO("Offboard enabled");
                // }  
            }
            if(drone_state_.mode == "AUTO.LAND" && landflag_ == true)
            {   
                ROS_INFO("AUTO.LAND enabled");   
                landflag_ = false;
            }

            break;            
        }

        case prometheus_msgs::ControlCommand::disarm:
        {
            arm_cmd.request.value = false;
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed "); 
            }
            break;            
        }

    }
    cmd_last_ = cmd_;
}

void CommandToFcu::send_xyz_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp)
{
    // mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.type_mask = 0b000111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void CommandToFcu::send_xyz_yaw_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, double yaw_sp)
{
    // mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void CommandToFcu::send_xyz_vel_yaw_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, double yaw_sp)
{
    // mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void CommandToFcu::send_vel_yaw_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d vel_sp, double yaw_sp)
{
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

// void CommandToFcu::send_xyz_vel_yaw_yawdot_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, double yaw_sp, double yawdot_sp)
// {
//     // mavros_msgs::PositionTarget pos_setpoint;

//     // 速度作为前馈项， 参见FlightTaskOffboard.cpp
//     // 2. position setpoint + velocity setpoint (velocity used as feedforward)
//     // 控制方法请见 PositionControl.cpp
//     pos_setpoint.type_mask = 0b000111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

//     pos_setpoint.coordinate_frame = 1;

//     pos_setpoint.position.x = pos_sp[0];
//     pos_setpoint.position.y = pos_sp[1];
//     pos_setpoint.position.z = pos_sp[2];
//     pos_setpoint.velocity.x = vel_sp[0];
//     pos_setpoint.velocity.y = vel_sp[1];
//     pos_setpoint.velocity.z = vel_sp[2];

//     pos_setpoint.yaw = yaw_sp;
//     pos_setpoint.yaw_rate = yawdot_sp;

//     setpoint_raw_local_pub.publish(pos_setpoint);
// }

// void CommandToFcu::send_xyz_vel_acc_yaw_yawdot_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, double yawdot_sp)
// {
//     // mavros_msgs::PositionTarget pos_setpoint;

//     // 速度作为前馈项， 参见FlightTaskOffboard.cpp
//     // 2. position setpoint + velocity setpoint (velocity used as feedforward)
//     // 控制方法请见 PositionControl.cpp
//     // pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

//     // pos_setpoint.coordinate_frame = 1;

//     // pos_setpoint.position.x = pos_sp[0];
//     // pos_setpoint.position.y = pos_sp[1];
//     // pos_setpoint.position.z = pos_sp[2];
//     // pos_setpoint.velocity.x = vel_sp[0];
//     // pos_setpoint.velocity.y = vel_sp[1];
//     // pos_setpoint.velocity.z = vel_sp[2];

//     // pos_setpoint.yaw = yaw_sp;

//     // setpoint_raw_local_pub.publish(pos_setpoint);
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommandsToFcu_node");
    ros::NodeHandle nh("~");
    CommandToFcu commandToFcu;
    commandToFcu.init(nh);
    ros::Rate rate(50.0);
    while(ros::ok())
    {
        ros::spinOnce();
        commandToFcu.run(0.02);//数值为运行频率的倒数
        rate.sleep();
    }
    return 0;
}