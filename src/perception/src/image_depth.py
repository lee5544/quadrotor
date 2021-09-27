#!/usr/bin/env python2
# coding=UTF-8
#视觉测距离（计算矩形框长宽）
import rospy, math
import numpy as np
import cv2
import message_filters
from sensor_msgs.msg import Image, PointCloud2,PointField
from cv_bridge import CvBridge, CvBridgeError    
from matplotlib import pyplot as plt

#目标hsv颜色空间区间
#室内
# lower = np.array([90,130,100])
# upper = np.array([150,220,255])


#室外
lower = np.array([100,80,80])
upper = np.array([130,200,255])



#轮廓提取参数 不需要调整
low_threshold =40   
ratio = 3
kernel_size = 3
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

area_threshold = 300


class  Mono_detector():
    def __init__(self):
        #self.publisher = rospy.Publisher("image_depth", vision_depth, queue_size = 1)
        self.pub_pointcloud = rospy.Publisher("/mono/pointcloud", PointCloud2, queue_size=1)
        self.pub_depthimg = rospy.Publisher("/mono/depth", Image, queue_size=1)
        self.subscribe = rospy.Subscriber("/camera/color/image_raw", Image , self.callback)
      
        self.bridge = CvBridge() 
       
        self.camera_intrinsic = np.array([[888.0543716729961, 0.0, 604.9191404405545],
                                                                           [ 0.0, 875.783377357364, 388.61391032308075],
                                                                           [ 0.0, 0.0, 1.0]])
        self.distortion = np.array([0.037058,-0.101511,0.003530,-0.008141])
        
    
    def callback(self,img_msg):
        #颜色空间转换 rgb -> hsv
        
        image = self.bridge.imgmsg_to_cv2(img_msg)
       # image = cv2.undistort(image,self.camera_intrinsic,self.distortion,None,self.camera_intrinsic)
        #plt.imshow(image),plt.show()  
        height= image.shape[0]
        width = image.shape[1]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    
        #plt.imshow(hsv),plt.show()  

        #根据hsv颜色空间初步把目标轮廓提取出来
        mask = cv2.inRange(hsv, lower, upper)
        #plt.imshow(mask),plt.show()

        #目标轮廓腐蚀膨胀操作，剔除hsv接近的干扰目标
        erosion=cv2.erode(mask,kernel,iterations=2)
        dilation=cv2.dilate( erosion,kernel ,iterations=4)
        erosion2=cv2.erode(dilation,kernel,iterations=2)

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
        data=[]
        #测距，根据目标的长（长完整情况下）或宽测量
        for n in range(0,cnt_num):            
            cnt=  contours[n]
            #计算目标面积并去除面积比较小的物体
            area= cv2.contourArea(cnt)
            if area<area_threshold:
                continue
           
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)     

            length_width = min(rect[1][0],rect[1][1])
            length_height = max(rect[1][0],rect[1][1])

            rrr =length_height / length_width
            print(rrr)
            
            if rrr>=8:
                dist =1700/length_height 
            elif rrr>2:
                #dist=length_width1828
                dist=210/length_width*math.sqrt(888**2+(640-rect[0][0])**2)/888
            else:
                continue

            #给相应的轮廓赋深度值
            final_img=np.zeros((height,width),dtype=np.float32)  
            cv2.drawContours(final_img,[cnt],0,(1,1,1),cv2.FILLED)  
            final_img=final_img*dist
            depth_img=np.maximum(final_img,depth_img)    
            #print(dist)       

            
            img = cv2.drawContours(image ,[box],-1,(0,0,255),2)
            cv2.putText(image, str(round(dist,2))+'m', (int(rect[0][0]), int(rect[0][1])), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2)
        
        cv2.imshow("视觉测距",image),cv2.waitKey(10)

    
        # cloud_msg=self.create_pointcloud2_msg(depth_img)
        # cloud_msg.header.stamp = img_msg.header.stamp
        # self.pub_pointcloud.publish(cloud_msg)

        cloud_msg=self.create_pointcloud2_msg(depth_img)
        cloud_msg.header.stamp = img_msg.header.stamp
        self.pub_pointcloud.publish(cloud_msg)



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
        for y in range(0,height,8):
            for x in range(0,width,8):
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
    rospy.init_node("mono_depth")
    Mono_detector()
    rospy.spin()
