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
from ambf_client import Client


if __name__ == '__main__':
    # coord = get_head_root_coord(self.arr,self.points)

    client = Client()
    client.connect()
    rate = rospy.Rate(50)
    root = client.get_obj_handle('Middle_1')
    print(root.get_pos())
    root_pos = np.array([root.get_pos().x, root.get_pos().y, root.get_pos().z])
    coord = np.append(root_pos, 1)
    print("root",coord)

    T_Cam_l,T_ECM_g = get_ECM()
    T_psml_g = get_PSM_l()
    print(np.linalg.inv(T_psml_g))
    # app = np.dot(np.linalg.inv(T_psml_g),coord) 
    # print("hh", app)
    # zz = np.linalg.inv(T_psml_g) @ coord
    # print("kk",zz)
    
    
    app_head_psml = get_app_head_psml(T_Cam_l,T_ECM_g,T_psml_g,coord)
    app_head_psml = np.array(app_head_psml, dtype=np.float32)
    print(app_head_psml)