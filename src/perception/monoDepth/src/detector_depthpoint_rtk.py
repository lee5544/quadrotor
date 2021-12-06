#!/usr/bin/env python2
# coding=UTF-8
# 视觉测距离（计算矩形框长宽）
import rospy
import math
import time
import cv2
import sys
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN

import convert

# 目标hsv颜色空间区间 red
lower_red_0 = np.array([0, 100, 10])
upper_red_0 = np.array([15, 255, 255])
lower_red_1 = np.array([170, 100, 10])
upper_red_1 = np.array([180, 255, 255])


# 轮廓提取参数 不需要调整
low_threshold = 40
ratio = 3
kernel_size = 3
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
kernel2 = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
area_threshold = 200
buff_num = 30


class Mono_detector():
    def __init__(self):
        self.pub_point = rospy.Publisher(
            "/obstacle_points", Float64MultiArray, queue_size=1)
        self.sub_odom = message_filters.Subscriber(
            "/mavros/local_position/odom", Odometry)
        self.sub_img = message_filters.Subscriber(
            "/camera/color/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_img], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.multi_callback)
        
        self.init = False
        self.keyframe = False
        self.data_buff = []
        self.ce_pre = np.array([0])
        
        self.bridge = CvBridge()
        self.estimator = DBSCAN(eps=0.5, min_samples=15, metric='euclidean')

        

    def multi_callback(self, odom_msg, img_msg):
        odom_ori = odom_msg.pose.pose.orientation
        odom_pos = odom_msg.pose.pose.position
        
        
        r,p,y= convert.Convert().quart_to_rpy(odom_ori.x, odom_ori.y, odom_ori.z,odom_ori.w)
        # y = y-0.25
        R= convert.Convert().eulerAnglesToRotationMatrix([r,p,y] )
        
        # R = convert.Convert().quaternion_to_rotation_matrix(
        #     np.array([odom_ori.x, odom_ori.y, odom_ori.z,odom_ori.w]))
        T = np.array([odom_pos.x, odom_pos.y, odom_pos.z]),
        R = np.mat(R)
        T = np.mat(T)

        if self.init == False:
            self.R_init = R
            self.T_init = T
            self.width = img_msg.width
            self.height = img_msg.height
            self.init = True

        if self.keyframe == False and self.init == True:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # image = cv2.resize(image,(640,480))
            ceter_point = self.vision_detector(image)

            if len(ceter_point) != 0:
                trans_point = convert.Convert().trans(
                    ceter_point, R, T, self.R_init, self.T_init)
                
                self.data_buff.append(trans_point)
                # print(trans_point )
                # print('*******************************************')
            if len(self.data_buff) >= buff_num:
                data = [token for st in self.data_buff  for token in st ]                   
                ce ,labels,num= self.MyDBSCAN(np.array(data))
                # print(ce)
                # print('*******************************************')
                # flags  = 0
                # for i in range(len(trans_point)):
                #     if labels[-i] ==-1:
                #         flags = flags  +1
                # if flags  ==len(trans_point):   
                #     self.data_buff.pop(-1)    
                # else:
                self.data_buff.pop(0)         
                if num != 0:    
                    ce_veh =   convert.Convert().world2uav(ce,R, T)   
                    ce_veh_avg = np.sum(ce_veh, axis=0)/num
                else:
                    ce_veh_avg = np.array([0,0,0])
                    # print(ce_veh_avg)
                    # print('*******************************************')
                if ce_veh_avg[0]>6 and ce_veh_avg[0] < 9 and num==1:
                    self.key_point =np.array(ce) #convert.Convert().trans_inv(ce, self.R_init, self.T_init, R, T)
                    self.R_key = R
                    self.T_key = T
                    self.keyframe = True
                    print("关键帧已找到")

                # if len(ce)!=0:
                #     ce_data = np.array([item[0] for item in ce])

                #     num = 0
                #     for item1 in ce_data:
                #         for item2 in self.ce_pre:
                #             if np.sum(item1-item2)**2<5 and item1[1]-np.array(T-self.T_init).flatten()[1]<12:
                #                 num+=1         
                #     self.ce_pre = ce_data

                #     if num ==len(ce):
                #         self.key_point = convert.Convert().trans_inv(ce_data, self.R_init, self.T_init, R, T)
                #         self.R_key = R
                #         self.T_key = T
                #         self.keyframe = True
                #         print("关键帧已找到")
      

        if self.keyframe == True:

            pub_point = self.key_point#convert.Convert().trans_inv(self.key_point, self.R_key, self.T_key, R, T)
            msg = Float64MultiArray()
            h_dim = MultiArrayDimension(
                label="height", size=pub_point.shape[0], stride=pub_point.shape[0]*pub_point.shape[1])
            w_dim = MultiArrayDimension(
                label="width",  size=pub_point.shape[1], stride=pub_point.shape[1])
            msg.layout = MultiArrayLayout(dim=[h_dim, w_dim], data_offset=0)
            msg.data = pub_point.reshape(-1).tolist()
            self.pub_point.publish(msg)
            print(pub_point)

    def vision_detector(self, image):
        img = cv2.filter2D(image, -1, kernel=kernel2)
        # img = image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 根据hsv颜色空间初步把目标轮廓提取出来
        mask_0 = cv2.inRange(hsv, lower_red_0, upper_red_0)
        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask = cv2.bitwise_or(mask_0, mask_1)
        # plt.imshow(mask),plt.show()

        # 目标轮廓腐蚀膨胀操作，剔除hsv接近的干扰目标
        erosion = cv2.erode(mask, kernel, iterations=1)
        dilation = cv2.dilate(erosion, kernel, iterations=4)
        erosion2 = cv2.erode(dilation, kernel, iterations=3)
        # plt.imshow(erosion2),plt.show()
        erosion2[0,:] = 0
        erosion2[self.height-1,:] = 0
        erosion2[:,0] = 0
        erosion2[:,self.width-1] = 0

        # 边缘提取+轮廓检测
        canny_edge = cv2.Canny(erosion2, low_threshold,
                               low_threshold*ratio, kernel_size)
        # plt.imshow(canny_edge),plt.show()
        contours = cv2.findContours(
            canny_edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[1]
        cnt_num = len(contours)

        # 测距，根据目标的长（长完整情况下）或宽测量
        center = []
        for n in range(0, cnt_num):
            cnt = contours[n]
            # 计算目标面积并去除面积比较小的物体
            rect = cv2.minAreaRect(cnt)
            area = cv2.contourArea(cnt)
            if area < area_threshold or rect[0][0] < 30 or rect[0][0] > self.width-30:
                continue

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            length_width = min(rect[1][0], rect[1][1])
            length_height = max(rect[1][0], rect[1][1])

            cnt_twodim = cnt.reshape(cnt.shape[0], cnt.shape[2])
            theta1 = (self.width/2-np.amax(cnt_twodim, axis=0)
                      [0]).astype(np.float32)*69/self.width/180*math.pi
            theta2 = (self.width/2-np.amin(cnt_twodim, axis=0)
                      [0]).astype(np.float32)*69/self.width/180*math.pi
            length_width = 2*length_width * \
                math.cos(theta1)*math.cos(theta2) / \
                (math.cos(theta1)+math.cos(theta2))

            rrr = length_height / length_width
            # print( "rrr",rrr)
           

            if rrr >= 7 and length_width <= 35:
                dist = 1910/length_height  # real 1850 980
            elif rrr > 2 and length_width >= 15:
                # dist=length_width1828
                dist = 220/length_width  # real 210  103
            else:
                continue

            # 给相应的轮廓赋深度值
            dist = dist-0.1
            center.append([rect[0][0], rect[0][1], dist])
            # center.append(dist)
            image = cv2.drawContours(image, [box], -1, (255, 0, 0), 1)
            cv2.putText(image, str(round(dist, 2))+'m', (int(rect[0][0]), int(
                rect[0][1])), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255, 0), 2)

        cv2.imshow("视觉测距", image), cv2.waitKey(10)
        # plt.imshow(image),plt.show()

        return center

    def MyDBSCAN(self, data_):
        # data= data.reshape(2,3)
        self.estimator.fit(data_)
        labels = self.estimator.labels_
        center_avg = []
        num = 0
        for item in np.unique(labels):
            if item != -1:
                point = data_[labels == item]
                avg = np.sum(point, axis=0)/len(point)
                conf = len(point)/buff_num
                # if conf >0.5:
                center_avg.append(avg)
                num +=1
        return center_avg,labels,num


if __name__ == "__main__":
    rospy.init_node("detector_rtk")
    Mono_detector()
    rospy.spin()
