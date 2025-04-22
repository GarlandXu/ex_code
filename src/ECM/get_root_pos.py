#! /usr/bin/env python

import time
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from vision_utilities import *
from std_msgs.msg import String

a =[]
b = []
        
class PointCloudSubscriber():
    def __init__(self):
        
        self.bridge = CvBridge()
        self.select_flag = 0
        self.click_approve = 0
        self.arr = np.empty((1,2))

        rospy.Subscriber('/ambf/env/cameras/camera_2/ImageData', Image, self.callback_img)
        rospy.Subscriber("/ambf/env/cameras/camera_2/DepthData",PointCloud2,self.callback_point) 
        rospy.Subscriber("approve_root_click",String,self.callback_switch)

        self.pub = rospy.Publisher('cut_goal', numpy_msg(Floats), queue_size=10)
        
    def callback_switch(self,msg):
        if msg.data == '1':
            self.click_approve = 1        
    
    def callback_img(self,data):

        if self.click_approve == 1:        
            self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")              
            self.image = self.cv_img

            cv2.namedWindow("image")
            cv2.setMouseCallback("image", self.on_EVENT_LBUTTONDOWN)
            cv2.imshow("image", self.image)
            cv2.waitKey(0)
        
        
    def callback_point(self, msg):
        if self.select_flag == 0:
            pass
        
        elif self.select_flag == 1:
            
            self.points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z")) 
            coord = get_head_root_coord(self.arr,self.points)

            T_Cam_l,T_ECM_g = get_ECM()
            T_psml_g = get_PSM_r()
            
            coord.append(1)
            app_head_psml = get_app_head_psml(T_Cam_l,T_ECM_g,T_psml_g,coord)

            self.app_head_psml = np.array(app_head_psml, dtype=np.float32)
            self.pub.publish(self.app_head_psml)
            
            print(app_head_psml)  
            rospy.signal_shutdown(' ')
        
            

    def on_EVENT_LBUTTONDOWN(self,event, x, y,flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            a.append(x)
            b.append(y)
            
            print('Coord of Root :',a[0],b[0])
            self.arr[0][0] = a[0]
            self.arr[0][1] = b[0]
            
            self.select_flag = 1
            
            
       
                    
def run_loop():
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        rate.sleep() 
        
if __name__ =='__main__':
    rospy.init_node("appendix_root_subscriber")
    node = PointCloudSubscriber()
    run_loop()  