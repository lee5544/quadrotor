#!/usr/bin/env python2
# coding=UTF-8

import numpy as np
import math

class  Convert():
    def __init__(self):
        self.R_base = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        self.T_base = 1
        self.camera_intrinsic = np.array([[927.058167, 0.0, 626.941797],
                                                                           [ 0.0, 924.753723,  350.936028],
                                                                           [ 0.0, 0.0, 1.0]])

        #  gazebo realsense
        # self.camera_intrinsic = np.array([[554, 0.0, 320],
        #                                                                    [ 0.0, 554, 240],
        #                                                                    [ 0.0, 0.0, 1.0]])

        # self.RR = np.array([[0.9843,-0.0197,0],[0.0783,0.9679,0],[0,0,1]])
        # self.RR = np.array([[0.9926,0.492,0],[0.0402,0.9621,0],[0,0,1]])
        # self.RR=np.array([[1,0,0],[0.081,1,0],[0,0,1]])
        




    def pixel2cam(self,pixel_point):
        fx = self.camera_intrinsic[0][0]
        fy = self.camera_intrinsic[1][1]
        cx = self.camera_intrinsic[0,2]
        cy = self.camera_intrinsic[1][2]

        cam_point = []
        for i in range(len(pixel_point)):
            z = pixel_point[i][2]
            x = (pixel_point[i][0]-cx)*z/fx
            y = (pixel_point[i][1]-cy)*z/fy
            cam_point.append([x,y,z])
        return np.mat(cam_point)


    
    def quaternion_to_rotation_matrix(self,q):  # x, y ,z ,w
        rot_matrix = np.array(
            [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
            [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
            [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
            dtype=q.dtype)
        return rot_matrix
    
    def quart_to_rpy(self,x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw           

        
    def eulerAnglesToRotationMatrix(sself,theta) :    
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
            
            
                        
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
                    
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
                        
                        
        R = np.dot(R_z, np.dot( R_y, R_x ))

        return R
            



    def cam2pixel(self,cam_point):
        fx = self.camera_intrinsic[0][0]
        fy = self.camera_intrinsic[1][1]
        cx = self.camera_intrinsic[0,2]
        cy = self.camera_intrinsic[1][2]

        pixel_point = []
        for i in range(len(cam_point)):
            u = fx*cam_point[i][0]/cam_point[i][2]+cx
            v = fy*cam_point[i][1]/cam_point[i][2]+cy
            pixel_point.append([u,v])
        
        return pixel_point   


    def quaternion_to_rotation_matrix(self,q):  # x, y ,z ,w
        rot_matrix = np.array(
            [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
            [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
            [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
            dtype=q.dtype)
        return rot_matrix


    def warp(self,depth_point,R_t,T_t,R_t_1,T_t_1):
        cam_point = self.pixel2cam(depth_point)
        # base_point = np.dot(self.R_base,cam_point.T)
        world_point = np.dot(np.dot(R_t,R_t_1.I),base_point)+T_t-T_t_1
        cam_point = np.dot(self.R_base,world_point)
        pixel_point = self.cam2pixel(np.array(cam_point.T))
        return pixel_point


    def trans(self,depth_point,R_t,T_t,R_t_1,T_t_1):
        # R_t = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        cam_point = self.pixel2cam(depth_point)
        
        base_point = np.dot(self.R_base,cam_point.T)

        # base_point = np.dot(self.RR,base_point )
        # RR = np.array([[0.9162,-0.2576,0],[0.2361,0.955,0],[0,0,1]])
        # R_t = np.dot(RR,R_t)
        world_point = np.dot(R_t,base_point).T+ T_t
        # world_point = np.dot(np.dot(R_t,R_t_1.I),base_point).T+ T_t-T_t_1  
        print(  world_point )
        return np.array(world_point ) 

    def trans_inv(self,world_point,R_t,T_t,R_t_1,T_t_1):
        world_point = np.mat(world_point).T
        world_point = np.dot(np.dot(R_t,R_t_1.I),world_point).T+ T_t-T_t_1  
        return np.array(world_point ) 

    def world2uav(self,world_point,R_t,T_t): 
        if world_point== []:
            return 0
        else:
            uav_point = np.dot(R_t.I,(world_point-T_t).T)
        return np.array(uav_point.T)
  



# def pixel2cam()
def main():
    T = Convert()  
    point  = np.array([[1,1,1],[2,2,2]]) 
    b=1




if __name__ == "__main__":  
    
    main()