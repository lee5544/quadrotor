#!/usr/bin/env python2
# coding=UTF-8
#视觉测距离（计算矩形框长宽）
import rospy, math
import time
import numpy as np
import cv2
import message_filters
from sensor_msgs.msg import Image, PointCloud2,PointField
from cv_bridge import CvBridge, CvBridgeError    
from matplotlib import pyplot as plt

#目标hsv颜色空间区间 red
lower_red_0 = np.array([0,100,10]) 
upper_red_0 = np.array([20,255,255])
lower_red_1 = np.array([170,100,10]) 
upper_red_1 = np.array([180,255,255])


#轮廓提取参数 不需要调整
low_threshold =40   
ratio = 3
kernel_size = 3
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
kernel2= np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])

area_threshold = 200


class  Mono_detector():
    def __init__(self):

        self.pub_pointcloud = rospy.Publisher("/pointcloud", PointCloud2, queue_size=1)
        self.pub_depthimg = rospy.Publisher("/depthimg", Image, queue_size=1)
        self.subscribe = rospy.Subscriber("/D435i/color/image_raw", Image , self.callback)
      
        self.bridge = CvBridge() 
        # self.template = self.template(480,640)
        # self.camera_intrinsic = np.array([[888.0543716729961, 0.0, 604.9191404405545],
        #                                                                    [ 0.0, 875.783377357364, 388.61391032308075],
        #                                                                    [ 0.0, 0.0, 1.0]])

       
       
    
    def callback(self,img_msg):
        #颜色空间转换 rgb -> hsv
        
        img = self.bridge.imgmsg_to_cv2(img_msg,"bgr8")
        # image = cv2.filter2D(img, -1, kernel=kernel2) 
        image = img

        # plt.imshow(image),plt.show()
        height= image.shape[0]
        width = image.shape[1]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #hsv[:,:,2] =hsv[:,:,2]+0.5*hsv[:,:,1]
        # plt.imshow(hsv),plt.show()


        #根据hsv颜色空间初步把目标轮廓提取出来

        mask_0 = cv2.inRange(hsv, lower_red_0, upper_red_0)
        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask = cv2.bitwise_or(mask_0, mask_1)
        # plt.imshow(mask),plt.show()

        #目标轮廓腐蚀膨胀操作，剔除hsv接近的干扰目标
        erosion=cv2.erode(mask,kernel,iterations=1)
        dilation=cv2.dilate( erosion,kernel ,iterations=4)
        erosion2=cv2.erode(dilation,kernel,iterations=3)

        #plt.imshow(erosion2),plt.show()
        
        erosion2[0]=0
        erosion2[height-1]=0
        for i in range(height-1):
            erosion2[i][0]=0
            erosion2[i][height-1]=0
        #边缘提取+轮廓检测
        canny_edge = cv2.Canny(erosion2, low_threshold, low_threshold*ratio, kernel_size)
        contours =cv2.findContours(canny_edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[1]
        cnt_num=len(contours)         
        
        depth_img=np.zeros((height,width),dtype=np.float32)
   
        #测距，根据目标的长（长完整情况下）或宽测量
        for n in range(0,cnt_num):            
            cnt=  contours[n]
            #计算目标面积并去除面积比较小的物体
            rect = cv2.minAreaRect(cnt)
            area= cv2.contourArea(cnt)
            if area<area_threshold or rect[0][0]<30 or rect[0][0]>width-30:
                continue
           
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)     



            length_width = min(rect[1][0],rect[1][1])
            length_height = max(rect[1][0],rect[1][1])

            cnt_twodim=cnt.reshape(cnt.shape[0],cnt.shape[2])

            b=np.amin(cnt_twodim, axis=0)[0]

           
            theta1= (width/2-np.amax(cnt_twodim, axis=0)[0]).astype(np.float32)*69/width/180*math.pi
            theta2= (width/2-np.amin(cnt_twodim, axis=0)[0]).astype(np.float32)*69/width/180*math.pi

            length_width = 2*length_width*math.cos(theta1)*math.cos(theta2)/(math.cos(theta1)+math.cos(theta2))

            rrr =length_height / length_width
            # print( rrr)
            
            if rrr>=8 and length_width<= 35:
                dist =980/length_height #real 1850  940
            elif rrr>1.8 and length_width>= 15:
                #dist=length_width1828
                dist=150/length_width      #real 210  150
            else:
                continue

            #给相应的轮廓赋深度值
            dist=dist-0.1
            final_img=np.zeros((height,width),dtype=np.float32)  
            cv2.drawContours(final_img,[cnt],0,(1,1,1),cv2.FILLED)  
            final_img=final_img*dist
            depth_img=np.maximum(final_img,depth_img)    
            
            img = cv2.drawContours(img ,[box],-1,(255,0,0),2)
            cv2.putText(img, str(round(dist,2))+'m', (int(rect[0][0]), int(rect[0][1])), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 0, 0), 2)
  
        cv2.imshow("视觉测距",img),cv2.waitKey(10)
        # plt.imshow(img),plt.show()

        depth_img=(depth_img<=20)*depth_img    
        # sparse_depth = np.minimum(depth_img, self.template)
        
        #cv2.convertScaleAbs
        # sparse_depth = sparse_depth.astype("uint16")
        # depth_img = cv2.resize(depth_img,(480,640))
        image_depth = self.bridge.cv2_to_imgmsg(depth_img,"32FC1" )
        image_depth.header=img_msg.header 
        self.pub_depthimg.publish(image_depth) 

        # cloud_msg=self.create_pointcloud2_msg(depth_img)
        # cloud_msg.header.stamp = img_msg.header.stamp
        # self.pub_pointcloud.publish(cloud_msg)
        # print(time.time())





    def template(self,height,width):
        #构造稀疏模板
        template = np.zeros((height,width),dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                if (y%4 ==0 and x%4 ==0):
                    template[y,x] = 255
                else:
                    continue
        return template
    def create_pointcloud2_msg(self,depth):

        fx = self.camera_intrinsic[0, 0]
        fy = self.camera_intrinsic[1, 1]
        cx = self.camera_intrinsic[0, 2]
        cy = self.camera_intrinsic[1, 2]


        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id ='map'
        height, width = depth.shape

        # Iterate images and build point cloud
        data = []
        for y in range(0,height,6):
            for x in range(0,width,6):
                if depth[y,x] != 0:
                    data.append([(x -cx)*depth[y, x]/fx,(y-cy) *depth[y, x]/fy,depth[y, x] ])

        data = np.array(data).reshape(-1,).astype(np.float32)
        b=1
      


        # Fields of the point cloud
        msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1) ]
        
        # Message data size
        msg.height = 1
        msg.width = len(data)/3

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(data)/3
        msg.is_dense = False
        msg.data = data.tobytes() 

        return msg




        
            
if __name__ == "__main__":    
    rospy.init_node("test")
    Mono_detector()
    rospy.spin()
