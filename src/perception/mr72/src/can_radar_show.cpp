/********纳雷串口雷达数据显示程序********/
//输入：自定义雷达数据类型
//输出：点云数据
/****************************************/

#include<ros/ros.h>
#include"can_rec/radar_pkg.h"
//#include <visualization_msgs/Marker.h>
#include<cmath>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>

class show{
    private:
        ros::NodeHandle n;
        //ros::Publisher show_pub;
        ros::Subscriber show_sub;
        ros::Publisher show_pub_point_cloud;
    public:
        show(){
            //show_pub = n.advertise<visualization_msgs::Marker>("serialmr72_show",10);
            show_pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("radar/point_cloud",1000);
            //show_pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("radar/point_cloud",1000);
            show_sub = n.subscribe("/can_radar_info",10,&show::radarinfocallback,this);
            //show_sub = n.subscribe("/serialmr72_info",10,&show::radarinfocallback,this);
        }
    
    void radarinfocallback(const can_rec::radar_pkg::ConstPtr& msg){
       //ROS_INFO("hello");
      sensor_msgs::PointCloud points2pub;
      points2pub.header.frame_id = "world";
      points2pub.header.stamp = msg->header.stamp;
      for(int i=0; i<msg->data.size(); i++)
      {
        geometry_msgs::Point32 p;
        p.x = msg->data[i].Objects_DistLat; 
        p.y = msg->data[i].Objects_DistLong;
        p.z = 0;
        points2pub.points.push_back(p);
      }
      show_pub_point_cloud.publish(points2pub);

     }

};


int main(int argc, char **argv){
    ros::init(argc,argv,"can_radar_show");
    ROS_INFO("This is the can_radar_show");
    show sw;
    ros::spin();
    return 0;
}
