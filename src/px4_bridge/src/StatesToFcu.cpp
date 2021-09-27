#include  "StatesToFcu.h"

StatesToFcu::StatesToFcu(ros::NodeHandle node)
{
    ROS_INFO("send state to fcu!");
    
    // 定位数据输入源 
    node.param<std::string>("input_source", input_source_, "t265");

    pos_offset[0]; pos_offset[1]; pos_offset[2];
    yaw_offset = 0;

    //  【订阅】t265估计位置
    // t265_sub = node.subscribe("/t265/odom/sample", 100, &StatesToFcu::t265Callback, this);
    t265_sub = node.subscribe("/camera/odom/sample", 100, &StatesToFcu::t265Callback, this);
     // 【订阅】gazebo仿真真值
    gazebo_sub = node.subscribe("/prometheus/ground_truth/p300_basic", 100, &StatesToFcu::gazeboCallback, this);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, this );
    gazebo_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, this );

    // 10秒定时打印，以确保程序在正确运行
    // timer = node.createTimer(ros::Duration(3.0), &StatesToFcu::timerCallback, this);
}

StatesToFcu::~StatesToFcu()
{
    ROS_INFO("Execture destructor function!");
}

void StatesToFcu::run()
{
    if(input_source_ == "t265"){
        vision_pub.publish(t256_vision_);
    }
    else if(input_source_ == "gazebo"){
        gazebo_pub.publish(gazebo_);
    }
}

void StatesToFcu::t265Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // if (msg->header.frame_id == "t265_odom_frame")
    if (msg->header.frame_id == "camera_odom_frame")
    {
        pos_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];

        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        // Euler_t265 = quaternion_to_euler(q_t265);
        // Euler_t265[2] = Euler_t265[2] + yaw_offset;
        // q_t265 = quaternion_from_rpy(Euler_t265);

        t256_vision_.header.stamp = msg->header.stamp;

        t256_vision_.pose.position.x = pos_t265[0];
        t256_vision_.pose.position.y = pos_t265[1];
        t256_vision_.pose.position.z = pos_t265[2];

        t256_vision_.pose.orientation.x = q_t265.x();
        t256_vision_.pose.orientation.y = q_t265.y();
        t256_vision_.pose.orientation.z = q_t265.z();
        t256_vision_.pose.orientation.w = q_t265.w();
    }
}

void StatesToFcu::gazeboCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_gazebo = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        // Euler_gazebo = quaternion_to_euler(q_gazebo);
        // Euler_gazebo[2] = Euler_gazebo[2] + yaw_offset;
        // q_gazebo = quaternion_from_rpy(Euler_gazebo);
        gazebo_.header.stamp = msg->header.stamp;

        gazebo_.pose.position.x = pos_gazebo[0];
        gazebo_.pose.position.y = pos_gazebo[1];
        gazebo_.pose.position.z = pos_gazebo[2];

        gazebo_.pose.orientation.x = q_gazebo.x();
        gazebo_.pose.orientation.y = q_gazebo.y();
        gazebo_.pose.orientation.z = q_gazebo.z();
        gazebo_.pose.orientation.w = q_gazebo.w();
    }
}

void StatesToFcu::timerCallback(const ros::TimerEvent &e)
{
    std::cout << "send module is working ..." << std::endl;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "StatesToFcu_node");
    ros::NodeHandle node("~");

    StatesToFcu statesToFcu(node);

    double frequency = 50;
    ros::Rate rate(frequency);

    while(ros::ok()){
        statesToFcu.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}