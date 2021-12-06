#include "CommandToFcuAtt.h"

CommandToFcuAtt::CommandToFcuAtt()
{}
CommandToFcuAtt::~CommandToFcuAtt()
{}

void CommandToFcuAtt::init(ros::NodeHandle node)
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
    setpoint_raw_attitude_pub = node.advertise<mavros_msgs::AttitudeTarget>( "/mavros/setpoint_raw/attitude", 10);

    set_mode_client = node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    drone_state_sub = node.subscribe("/prometheus/drone_state", 10, &CommandToFcuAtt::dronestateCallback, this);
    command_sub = node.subscribe("/prometheus/control_command", 100, &CommandToFcuAtt::commandCallback, this);
}

void CommandToFcuAtt::commandCallback(const prometheus_msgs::ControlCommand::ConstPtr &msg)
{
    cmd_ = *msg;
    // run(0.02);//数值为运行频率的倒数
}

void CommandToFcuAtt::dronestateCallback(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_state_ = *msg;
}

bool CommandToFcuAtt::check_failsafe()
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

void CommandToFcuAtt::run(double dt)
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
                if(cmd_last_.mode != prometheus_msgs::ControlCommand::idle)
                {
                    ROS_INFO("Wait for ARMMING and OFFBOARD!");
                }
                pos_setpoint.type_mask = 0x4000;
                pos_setpoint.coordinate_frame = 1;
                setpoint_raw_local_pub.publish(pos_setpoint);

            break;            
        }

        case prometheus_msgs::ControlCommand::takeoff:
        {
                if (cmd_last_.mode != prometheus_msgs::ControlCommand::takeoff)//防止多次起飞
                {
                    ROS_INFO("TAKEOFF");                    
                    
                    if(cmd_.reference_state.position_ref[2] > 0.5)//判断用户输入起飞高度是否有效
                    {
                        takeoff_height_ = cmd_.reference_state.position_ref[2];
                        std::cout<<"    takeoff height is :     "<< takeoff_height_ << std::endl;
                    }
                    
                    pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                    yaw_sp = cmd_.reference_state.yaw_ref;//当前航向角

                    // cmd_.reference_state.position_ref = {pos_sp[0], pos_sp[1], pos_sp[1]};
                    // cmd_.reference_state.yaw_ref = yaw_sp;
                    std::cout<<"    the takeoff target is :     "<< pos_sp.transpose() << "   " << yaw_sp*53.7 << std::endl;
                }
                send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);  

            break;            
        }

        case prometheus_msgs::ControlCommand::hold:
        {
            if (cmd_last_.mode != prometheus_msgs::ControlCommand::hold)//仅仅记录一次当前位置
            {
                ROS_INFO("HOLD");
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                yaw_sp = cmd_.reference_state.yaw_ref;
                std::cout<<"    The hold target is :     "<< pos_sp.transpose() << "   " << yaw_sp*53.7 << std::endl;                                       
            }
            send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);

            break;            
        }

        case prometheus_msgs::ControlCommand::move:
        {
            if (cmd_last_.mode != prometheus_msgs::ControlCommand::move)//仅仅记录一次当前位置
                ROS_INFO("MOVE");
            //  获取目标点
            //  根据模式，选择相应的控制模式
            if( cmd_.reference_state.move_mode  == prometheus_msgs::PositionReference::xyz )
            {
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                send_xyz_setpoint(pos_setpoint, pos_sp);
            }
            else if( cmd_.reference_state.move_mode  == prometheus_msgs::PositionReference::xyz_yaw )//已经测试
            {
                pos_sp << cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2];
                yaw_sp = cmd_.reference_state.yaw_ref;
                send_xyz_yaw_setpoint(pos_setpoint, pos_sp, yaw_sp);
            }
            else if( cmd_.reference_state.move_mode == prometheus_msgs::PositionReference::trajectory)
            {
                std::cout << " cal att " << std::endl;
                Eigen::Vector3d cur_pos(drone_state_.position[0], drone_state_.position[1], drone_state_.position[2]); 
                Eigen::Vector3d cur_vel(drone_state_.velocity[0], drone_state_.velocity[1], drone_state_.velocity[2]);
                Eigen::Vector3d cur_acc(drone_state_.accleration[0], drone_state_.accleration[1], drone_state_.accleration[2]);
                Eigen::Vector3d tar_pos(cmd_.reference_state.position_ref[0], cmd_.reference_state.position_ref[1], cmd_.reference_state.position_ref[2]);
                Eigen::Vector3d tar_vel(cmd_.reference_state.velocity_ref[0], cmd_.reference_state.velocity_ref[1], cmd_.reference_state.velocity_ref[2]);
                Eigen::Vector3d tar_acc(cmd_.reference_state.acceleration_ref[0], cmd_.reference_state.acceleration_ref[1], cmd_.reference_state.acceleration_ref[2]);
                yaw_sp = cmd_.reference_state.yaw_ref;

                Eigen::Vector3d  thrust_temp = controller_.pos_controller(cur_pos, cur_vel, cur_acc, tar_pos, tar_vel, tar_acc);
                
                double all_thrust; Eigen::Quaterniond att_sp;
                controller_.thrust2quaternion(thrust_temp, yaw_sp, all_thrust, att_sp);

                send_att_setpoint(all_thrust, att_sp);
            }
            // else if( cmd_.move_mode  == xyz_vel_acc_yaw )
            // {
            //     send_xyz_vel_acc_yaw_yawdot_setpoint(pos_setpoint, pos_sp, vel_sp, acc_sp, yaw_sp);
            // }
            break;           
        }

        case prometheus_msgs::ControlCommand::land:
        {
            if (cmd_last_.mode != prometheus_msgs::ControlCommand::land)//记录当前位置，只进入一次
            {
                ROS_INFO("LANDing");
                pos_sp[0] = drone_state_.position[0];
                pos_sp[0] = drone_state_.position[1];
                yaw_sp = drone_state_.attitude[2];//当前航向角
            }


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

void CommandToFcuAtt::send_xyz_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp)
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

void CommandToFcuAtt::send_xyz_yaw_setpoint(mavros_msgs::PositionTarget pos_setpoint, Eigen::Vector3d pos_sp, double yaw_sp)
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

//发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
void CommandToFcuAtt::send_att_setpoint(const double thrust_setpoint, const Eigen::Quaterniond attitude_setpoint)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    att_setpoint.orientation.x = attitude_setpoint.x();
    att_setpoint.orientation.y = attitude_setpoint.y();
    att_setpoint.orientation.z = attitude_setpoint.z();
    att_setpoint.orientation.w = attitude_setpoint.w();

    att_setpoint.thrust = thrust_setpoint;

    setpoint_raw_attitude_pub.publish(att_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg] "<<euler_fcu_target[1] * 180/M_PI << " [deg] "<< euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
    // cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommandToFcuAtt_node");
    ros::NodeHandle nh("~");
    CommandToFcuAtt CommandToFcuAtt;
    CommandToFcuAtt.init(nh);
    
    ros::Rate rate(50.0);
    while(ros::ok())
    {
        ros::spinOnce();
        CommandToFcuAtt.run(0.02);//数值为运行频率的倒数
        rate.sleep();
    }
    return 0;
}