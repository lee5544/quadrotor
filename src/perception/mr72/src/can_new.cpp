 /******CAN口毫米波雷达数据采集程序******/
//输入：无（实际是CAN总线的数据流）
//输出：自定义的雷达数据类型
//最后修改日期：2021-10-22
/*************************/
#include<iostream>
#include<math.h>
// #include<stdint.h>
// #include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<net/if.h>
#include<sys/ioctl.h>
#include<sys/socket.h>
#include<linux/can.h>
#include<linux/can/raw.h>


#include<ros/ros.h>
#include"can_rec/radar.h"
#include"can_rec/radar_pkg.h"

//#define DIST_THRESHOLD  100 //距离阈值
#define PI 3.1416

//一些需要用到的阈值
#define Dist_Over_Close 18//地面点的滤除阈值
#define Angle_Thre 12 //预警的角度阈值，单位：度


using  namespace std;

int main(int argc,char **argv)
{	//initialize the node
	ros::init(argc,argv,"can_rec_node");
	//create the manager of the node
	ros::NodeHandle n;
	ros::Publisher radar_info_pub = n.advertise<can_rec::radar_pkg>("/can_radar_info",10);
    can_rec::radar radar_msg;
    can_rec::radar_pkg array_msg;
	int s, nbytes;
    int i=0,count=0;
    bool flag = 0;
	//float cam_coef = 0.001082; //640*480
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr); //指定 can0 设备
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can0 绑定

	while(ros::ok())
	{	
		nbytes = read(s,&frame,sizeof(frame)); //receive the message

		if(nbytes>0)
		{
			if(flag ==0 && frame.can_id==0x60A){
					array_msg.header.stamp = ros::Time::now(); //以0x60A对应的时间戳作为整组雷达数据的时间戳
					count = frame.data[0];
					//cout<<"the rec num is: "<<count<<endl;
					//在检测到有目标时，将进行目标信息解算，否则继续探测下一个0x60A
					if(count==0){
						flag = 0;
					}
					else{
						flag = 1;
					}
			}
			//printf("It's jsut okay.\n");
			if(flag==1 && frame.can_id==0x60B){
				radar_msg.header.stamp = array_msg.header.stamp; //同一组雷达数据的时间戳理论上相同
				/***解算雷达探测到的物理量***/
				radar_msg.Objects_ID = frame.data[0]; //目标ID
				radar_msg.Objects_DistLong = (frame.data[1]*32+(frame.data[2] >> 3))*0.2-500; //目标纵向距离
				radar_msg.Objects_DistLat = ((frame.data[2]&0x07)*256+frame.data[3])*0.2-204.6; //目标横向距离
				radar_msg.Objects_VrelLong = (frame.data[4]*4+(frame.data[5]>>6))*0.25-128; //目标纵向速度
				radar_msg.Objects_VrelLat = ((frame.data[5]&0x3F)*8+(frame.data[6]>>5))*0.25-64; //目标横向速度
				radar_msg.Objects_DynProp = frame.data[6]&0x07;//目标动态属性 默认是为0
				radar_msg.Section_Number = (frame.data[6]>>3)&0x03;  //目标扇区编号
				radar_msg.Objects_RCS = frame.data[7]*0.5-64;//目标Rcs 默认输出是0
				radar_msg.azimuth = atan2f(radar_msg.Objects_DistLat,radar_msg.Objects_DistLong);//目标方位角（与y轴的夹角），单位为弧度
				//radar_msg.azimuth = atan2f((38.164*cam_coef+1.4261*tanf(radar_msg.azimuth)),1);//修正后的弧度值
				radar_msg.Objects_Dist = sqrt(radar_msg.Objects_DistLong *
				radar_msg.Objects_DistLong +radar_msg.Objects_DistLat *
				radar_msg.Objects_DistLat);//距离值，单位m
				radar_msg.Objects_Vrel = radar_msg.Objects_VrelLong * cosf(radar_msg.azimuth)+radar_msg.Objects_VrelLat * sinf(radar_msg.azimuth);//单位m/s
				radar_msg.obj_angle = (radar_msg.azimuth * 180 )/PI; // 弧度值转角度值，角度
				//radar_msg.Objects_Dist=radar_msg.Objects_Dist*cos(radar_msg.azimuth); //dist*cos(相机坐标系中的z轴)

				//if(radar_msg.Objects_Dist< DIST_THRESHOLD){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
				//	array_msg.data.push_back(radar_msg);}

				//仅对满足条件的数据点进行装载
				if(radar_msg.Objects_Dist>Dist_Over_Close && radar_msg.obj_angle<Angle_Thre && radar_msg.obj_angle>-(Angle_Thre)){
					array_msg.data.push_back(radar_msg); //数据的装载
				}

				i++;      //每完成一次数据解算，记录一下次数
				//解算完成的数据点数达到0x60A中反馈的当前目标数时，发布雷达数据
				if(i==count){
					i=0;
					flag=0;
					radar_info_pub.publish(array_msg); 
					//cout<<"the send num is: "<<array_msg.data.size()<<endl<<endl;
					array_msg.data.clear();
				}
			}
		}
	}
	close(s);
	return 0;

}
