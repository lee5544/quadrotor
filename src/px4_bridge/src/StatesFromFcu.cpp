#include "StatesFromFcu.h"

StatesFromFcu::StatesFromFcu() {}

StatesFromFcu::~StatesFromFcu() {}

void StatesFromFcu::init(ros::NodeHandle& state_nh)
{     
    ROS_INFO("read states from fcu!");
    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
    state_sub = state_nh.subscribe("/mavros/state", 10, &StatesFromFcu::stateCallback, this);

    odom_sub = state_nh.subscribe("/mavros/local_position/odom", 10, &StatesFromFcu::localOdomCallback, this);
    
    state_pub = state_nh.advertise<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, this);
}

void StatesFromFcu::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    drone_state_.connected = msg->connected;
    drone_state_.armed = msg->armed;
    drone_state_.mode = msg->mode;
    // ROS_INFO("current state: ", current_state.mode ); 
}

void StatesFromFcu::localOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    drone_state_.position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    drone_state_.velocity = {msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z};
    drone_state_.attitude_q = msg->pose.pose.orientation;
    Eigen::Vector3d ans;
    ans = quaternion_to_euler(drone_state_.attitude_q);
    drone_state_.attitude = {ans[0], ans[1], ans[2]};

    drone_state_.header.stamp = ros::Time::now();
    state_pub.publish(drone_state_);
    // std::cout << local_pos.transpose() << "   " << local_euler.transpose() << std::endl;
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
Eigen::Vector3d StatesFromFcu::quaternion_to_euler(geometry_msgs::Quaternion &q)
{
    float quat[4];
    quat[0] = q.w;
    quat[1] = q.x;
    quat[2] = q.y;
    quat[3] = q.z;

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "StatesFromFcu_node");
    ros::NodeHandle node("~");

    StatesFromFcu states;
    states.init(node);

    ros::Rate rate(20.0);
    // Eigen::Vector3d pos, Euler;
    // Eigen::Quaterniond q;
    
    while( ros::ok() )
    {
        // std::cout << drone_state_.local_pos.transpose() << "    " << drone_state_.local_euler.transpose()<< std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
