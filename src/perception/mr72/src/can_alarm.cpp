 /******雷达预警程序******/
//输入：雷达预警滤波数据
//输出：预警标志位
//最后修改日期：2021-09-27
/*************************/

#include<ros/ros.h>
#include"can_rec/radar.h"
#include"can_rec/radar_pkg.h"
#include"can_rec/trigger.h"
#include<algorithm>
#include<iostream>
using namespace std;

//#define Appear_Num 200 //预警范围内障碍物出现次数的阈值 
//#define Angle_Thre 12 //预警的角度阈值，单位：度
#define Tracking_Dist  10//预警监测区间内，连续追踪并引发报警的距离
#define Dist_ThreH 40 //预警距离的上限
#define Dist_ThreL 25//预警距离的下限

//#define Dist_Over_Close 10//地面点的滤除阈值，这一部分的阈值设置已移至can_alarm_filter_v2.cpp中

bool send_alarming = 1; // 发送标志位，值为1表示允许发送预警信号，值为0表示允许发送预警解除信号

class alarming{
    private:
        ros::NodeHandle nh;
        ros::Publisher alarm_pub;
        ros::Subscriber alarm_sub;
        can_rec::trigger stimulate; //预警触发信号
        vector<can_rec::radar> tracking_init,tracking_follow; //用于在预警监测区间内存储雷达点的容器
        vector<can_rec::radar>::const_iterator iter_init,iter_follow; //指向容器中距离最小的元素
        bool init_flag = 1; //判断是否是第一次在监测区间内探测到点
    public:
        alarming(){
            alarm_pub = nh.advertise<can_rec::trigger>("/radar/alarming",10);
            alarm_sub = nh.subscribe("/can_radar_info",10,&alarming::alarminfocallback,this);
            //remove_alarm_sub = nh.subscribe();
        }
        //预警监测程序的回调函数
        void alarminfocallback(const can_rec::radar_pkg::ConstPtr& msg);
        //预警解除程序的回调函数（预留）
        //void alarmingremovecallback(const bool);
};

//预警函数
 void alarming::alarminfocallback(const can_rec::radar_pkg::ConstPtr& msg){
        //vector<can_rec::radar> inspector;
        //vector<can_rec::radar>::iterator ins_iter;
        //inspector = msg->data;
        //ins_iter = min_element(inspector.begin(),inspector.end(),[](const can_rec::radar &a, const can_rec::radar &b ){return a.Objects_Dist<b.Objects_Dist;});
        //cout<<"The min dist is "<<(*ins_iter).Objects_Dist<<"m"<<endl;
        //cout<<"The min velocity is "<<(*ins_iter).Objects_Vrel<<"m/s"<<endl;
        for(int i=0;i<msg->data.size();i++){
            //如果已经是预警状态，不再处理雷达数据
            if(stimulate.flag ==1){
                break;
            }
            //如果出现低于阈值下限的点，则立即触发预警（msg中应当已不包含距离特别近的地面杂波）
            if(msg->data[i].Objects_Dist<Dist_ThreL){
                stimulate.header.stamp = ros::Time::now();
                stimulate.flag = 1;
                cout<<"The alarming distance is "<<msg->data[i].Objects_Dist<<"m"<<endl;
                break;
            }
            //如果出现处于距离阈值范围内的点，则监测障碍物移动的距离
            //cout<<msg->data[i].Objects_Dist<<endl;
            if(msg->data[i].Objects_Dist>=Dist_ThreL && msg->data[i].Objects_Dist <=Dist_ThreH){
                //cout<<"yeah"<<endl;
                //如果是第一次在区间内监测到点，则装到init容器中
                if(init_flag ==1){
                    tracking_init.push_back(msg->data[i]);
                }
                //如果不是第一次监测到点，则装载到follow容器中
                else{
                    tracking_follow.push_back(msg->data[i]);
                }
            }
     
        }

        //如果init容器初始化完毕，令初始标志位为假
        if(tracking_init.size()){
            init_flag = 0;
        }
        //在监测区间内进行距离追踪
        if(stimulate.flag == 0 && tracking_init.size() && tracking_follow.size()){
            iter_init = min_element(tracking_init.begin(),tracking_init.end(),[](const can_rec::radar &a, const can_rec::radar &b ){return a.Objects_Dist<b.Objects_Dist;});
            iter_follow = min_element(tracking_follow.begin(),tracking_follow.end(),[](const can_rec::radar &a, const can_rec::radar &b ){return a.Objects_Dist<b.Objects_Dist;});
            //cout<<"The min dist is "<<(*iter_follow).Objects_Dist<<"m"<<endl;
            //cout<<"The min velocity is "<<(*iter_follow).Objects_Vrel<<"m/s"<<endl;
            //如果后来follow中探测到的物体距离比之前init中的距离还远，那么可能在预警开始阶段出现错误探测，需要用follow重置init
            if((*iter_follow).Objects_Dist>(*iter_init).Objects_Dist){
                tracking_init.assign(tracking_follow.begin(),tracking_follow.end());
            }
            //如果follow中的距离相对init中的距离移动超过了监测距离阈值，则触发预警
            if((*iter_init).Objects_Dist-(*iter_follow).Objects_Dist>=Tracking_Dist){
                stimulate.header.stamp = ros::Time::now();
                stimulate.flag = 1;
                cout<<"The alarming distance is "<<(*iter_follow).Objects_Dist<<"m"<<endl;
            }
            tracking_follow.clear();
        }
        //当预警触发时发送预警信号，并同时置send_alarming标志位为0
        if(stimulate.flag == 1 && send_alarming==1){
            //设置规划的最大速度和最大加速度,暂且预留
            //stimulate.Max_Vel = 1;
            //timulate. Max_Acc = 0.5;
            alarm_pub.publish(stimulate);
            send_alarming =0;
            tracking_init.clear(); //预警完成后，清空init容器
            tracking_follow.clear(); //把follow容器也清空
            init_flag = 1; //允许重新初始化init容器
            cout<<"Warning! The UAV has to slow down!"<<endl;
            //exit(0);
        }

        }
    

/*****预留函数模块*****/
//预警解除回调函数
// void alarming::alarmingremovecallback(const bool){
//     send_alarming = 1; //重新允许发送预警信号
//     stimulate.header.stamp = ros::Time::now();
//     stimulate.flag = 0;
//     //设置规划的最大速度和最大加速度
//     stimulate.Max_Vel = 2;
//     stimulate. Max_Acc = 1;
//     alarm_pub.publish(stimulate);
//     cout<<"Now, the UAV is allowed to accelerate." <<endl;
// }
/************************/

int main(int argc, char **argv){
    ros::init(argc,argv,"alarming_node");
    ROS_INFO("This is the alarming program.");
    alarming al;
    ros::spin();
    return 0;
}