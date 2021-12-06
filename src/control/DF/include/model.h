/*
 * @Name:         model.h
 * @Author:       yong
 * @Date: 2021-11-24 15:07:01
 * @LastEditors:         yong
 * @LastEditTime: 2021-11-25 10:59:58
 * @Description:      
 * @Subscriber:       
 * @Publisher:        
 */

#include <math.h>
#include <iostream>

#include <Eigen/Eigen>

class MavModel{
    public: 
        MavModel();
        ~MavModel();
        //环境参数
        Eigen::Vector3d grivate;// 重力加速度

        //无人机参数
        double mav_mass;

        //电机参数
        double MOTOR_P1 = -0.00003098;
        double MOTOR_P2 = 0.00141;
        double MOTOR_P3 = -0.02133;
        double MOTOR_P4 = 0.1832;
        double MOTOR_P5 = 0.0001481;        
        
        
        //限制项
        double max_fb_acc;
        double max_theta;

    private: 
    
};

MavModel::MavModel()
{
    //环境参数
    grivate << 0.0, 0.0, 9.8;// 重力加速度

    //无人机参数
    mav_mass = 2.0;

    //电机参数
    MOTOR_P1 = -0.00003098;
    MOTOR_P2 = 0.00141;
    MOTOR_P3 = -0.02133;
    MOTOR_P4 = 0.1832;
    MOTOR_P5 = 0.0001481;        
    
    //限制项
    max_fb_acc = 9.0;
    max_theta = M_PI / 4;
}

MavModel::~MavModel(){}