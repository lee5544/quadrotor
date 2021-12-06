/***************************************************************************************************************************
* pos_controller_DF.h
*
* Author: GYL
*
* Update Time: 2021.9.4
*
* Introduction:  利用四旋翼无人机的微分平坦特性进行位置环控制解算
***************************************************************************************************************************/
#ifndef pos_controller_DF_h
#define pos_controller_DF_h

#include <math.h>
#include <iostream>

#include <Eigen/Eigen>

#include "model.h"
#include "math_utils.h"

using namespace std;

class pos_controller_DF
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:
        pos_controller_DF();
        ~pos_controller_DF();

        MavModel model;

        //标志位
        bool control_yaw;
        // 初始目标值
        double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
        // 目标位置、速度、加速度
        Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_;
        Eigen::Vector3d targetPos_prev_, targetVel_prev_;
        // 预设的期望偏航角，如果velocity_yaw_设置为true，则此设置无效
        double mavYaw_;
        // 无人机状态信息 - 位置、速度、角速度
        Eigen::Vector3d mavPos_, mavVel_, mavRate_;

        // 无人机姿态，期望姿态
        Eigen::Vector4d mavAtt_, q_des;
        // 期望角速度及推力
        Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
        // rotor drag
        double dx_, dy_, dz_;
        Eigen::Vector3d D_;
        // 控制参数
        double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
        Eigen::Vector3d Kpos_, Kvel_;
        //新增积分
        Eigen::Vector3d _integral, _Ki; 
        double _dt=1/100.0, _integral_max=1.0;


        //Printf the PID parameter
        void printf_param();
        void printf_result();
        
        // Position control main function 
        Eigen::Vector3d pos_controller(const Eigen::Vector3d cur_pos, const Eigen::Vector3d cur_vel, const Eigen::Vector3d cur_acc,
                const Eigen::Vector3d tar_pos, const Eigen::Vector3d tar_vel, const Eigen::Vector3d tar_acc);
        Eigen::Vector3d thrustToThrottle(const Eigen::Vector3d& thrust_sp);
        double Single_thrustToThrottle(const double& thrust_sp);
        void thrust2quaternion(const Eigen::Vector3d &thrust, const double &yaw, 
                    double &all_thrust, Eigen::Quaterniond &att);

    private:
        static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };

};

pos_controller_DF::pos_controller_DF()
{

    control_yaw = true;
    mavYaw_ = 0.0;

    // 初始目标值
    targetPos_ << 0.0, 0.0, 1.0;  // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;

    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;


    Kpos_ << 8.0, 8.0, 10.0;
    Kvel_ << 1.5, 1.5, 3.3;
    D_ << 0.0, 0.0, 0.0;
    _integral<<0.0,0.0,0.0;
    _Ki<<0.0,0.0,1.0;    //只对Z轴进行积分
}

pos_controller_DF::~pos_controller_DF(){}

//位置环解算，返回三维推力
Eigen::Vector3d pos_controller_DF::pos_controller(const Eigen::Vector3d cur_pos, const Eigen::Vector3d cur_vel, const Eigen::Vector3d cur_acc,
                    const Eigen::Vector3d tar_pos, const Eigen::Vector3d tar_vel, const Eigen::Vector3d tar_acc)
{
    Eigen::Vector3d pos_error =  tar_pos - cur_pos;
    Eigen::Vector3d vel_error = tar_vel - cur_vel ;

    // 加速度 - 反馈部分
    // 根据位置、速度误差计算，同时设置限幅，即max_fb_acc_
    // Kpos_ 是三维向量，Kpos_.asDiagonal()变成对角矩阵
    Eigen::Vector3d a_fb =Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

    if (a_fb.norm() > model.max_fb_acc)// Clip acceleration if reference is too large
    {
        a_fb = (model.max_fb_acc / a_fb.norm()) * a_fb;
    }

     _integral(2)+= _Ki(2) * pos_error(2) * _dt; //  只针对 z轴位置误差进行积分
       
    if(_integral(2) > _integral_max){//积分项限幅
        _integral(2) = _integral_max;
    }
    else if(_integral(2) < -_integral_max){
        _integral(2) = -_integral_max;
    }
    std::cout<<"积分项："<< _integral(2)<<std::endl;

    a_fb=a_fb+_integral;

    // 期望加速度 = 加速度反馈部分 + 加速度参考值 - rotor drag - 重力加速度
    Eigen::Vector3d a_des = a_fb + targetAcc_  + model.grivate;
    
  //Limit control angle to 45 degree 俯仰和翻滚角不会超过45度
  Eigen::Vector3d  a_limit_des = a_des;
  if(a_limit_des(0)>=(a_limit_des(2)*tan(model.max_theta)))  a_limit_des(0)=a_limit_des(2)*tan(model.max_theta);
  if(a_limit_des(1)>=(a_limit_des(2)*tan(model.max_theta)))  a_limit_des(1)=a_limit_des(2)*tan(model.max_theta);
  
  Eigen::Vector3d  thrust = a_limit_des * model.mav_mass / 4;

  return thrust;
}

Eigen::Vector3d pos_controller_DF::thrustToThrottle(const Eigen::Vector3d& thrust_sp)
{
    Eigen::Vector3d throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线
    for (int i=0; i<3; i++)
    {
        throttle_sp[i] = model.MOTOR_P1 * pow(thrust_sp[i],4) + model.MOTOR_P2 * pow(thrust_sp[i],3) 
                    + model.MOTOR_P3 * pow(thrust_sp[i],2) + model.MOTOR_P4 * thrust_sp[i] + model.MOTOR_P5;
       
        // throttle_sp[i] =MOTOR_K*thrust_sp[i]+MOTOR_b;
        // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
        // throttle_sp[i] = thrust_sp[i]/20；
    }
    return throttle_sp; 
}

double pos_controller_DF::Single_thrustToThrottle(const double& thrust_sp)
{
    double throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线

 throttle_sp = model.MOTOR_P1 * pow(thrust_sp,4) + model.MOTOR_P2 * pow(thrust_sp,3) 
                    + model.MOTOR_P3 * pow(thrust_sp,2) + model.MOTOR_P4 * thrust_sp + model.MOTOR_P5;
       
    return throttle_sp; 
}

void pos_controller_DF::thrust2quaternion(const Eigen::Vector3d &thrust, const double &yaw, 
                    double &all_thrust, Eigen::Quaterniond &att)
{
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;   //旋转矩阵

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;//std::cos(0.0), std::sin(0.0), 0.0;

    zb_des = thrust / thrust.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);

    Eigen::Quaterniond quat(rotmat);  //四元数
    att = quat;

   // Eigen::Vector3d euler_angles = rotmat.eulerAngles(2,1,0);
    Eigen::Vector3d euler_angles;
    rotation_to_euler(rotmat, euler_angles);

    cout << "Desired euler [R P Y]: "<< euler_angles[0]* 180/M_PI <<" [deg] " << euler_angles[1]* 180/M_PI <<" [deg] "<< euler_angles[2]* 180/M_PI <<" [deg] "<< endl;
    cout << "Desired Thrust: "<< thrust.norm()<< endl;

    //拉力转油门量
    Eigen::Vector3d vector_throttle = thrustToThrottle(thrust);
    all_thrust = Single_thrustToThrottle(thrust.norm());

    std::cout<<"设置的油门是:"<<all_thrust<<std::endl;
}

void pos_controller_DF::printf_result()
{
    // //固定的浮点显示
    // std::cout.setf(ios::fixed);
    // //左对齐
    // std::cout.setf(ios::left);
    // // 强制显示小数点
    // std::cout.setf(ios::showpoint);
    // // 强制显示符号
    // std::cout.setf(ios::showpos);

    // std::cout<<setprecision(2);
}

// 【打印参数函数】
void pos_controller_DF::printf_param()
{
    // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>微分平坦控制器参数 <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    // std::cout <<"Quad_MASS : "<< Quad_MASS << std::endl;

    // std::cout <<"control_yaw : "<< control_yaw << std::endl;
    // std::cout <<"max_fb_acc_ : "<< max_fb_acc_ << std::endl;

    // std::cout <<"Kpos_x_ : "<< Kpos_x_<< std::endl;
    // std::cout <<"Kpos_y_ : "<< Kpos_y_ << std::endl;
    // std::cout <<"Kpos_z_ : "<< Kpos_z_ << std::endl;

    // std::cout <<"Kvel_x_ : "<< Kvel_x_<< std::endl;
    // std::cout <<"Kvel_y_ : "<< Kvel_y_<< std::endl;
    // std::cout <<"Kvel_z_ : "<< Kvel_z_ << std::endl;


}


#endif
