import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from vision_utilities import *
from mrcnn import model as modellib
from mrcnn import visualize

MODEL_DIR = './logs'


class PointCloudSubscriber(object):
    def __init__(self):
        
        self.bridge = CvBridge()
        self.arr = np.empty((2,2))
        self.count = 0
        self.open = 0

        rospy.Subscriber('/ambf/env/cameras/camera_2/ImageData', Image, self.callback_img)
        rospy.Subscriber("/ambf/env/cameras/camera_2/DepthData",PointCloud2,self.callback_point) 
        self.pub = rospy.Publisher('grasp_goal', numpy_msg(Floats), queue_size=10)
        
        
    def callback_img(self,data):
        
        self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")              
        self.image = self.cv_img        
        
        self.config = Appendix_Config()
        self.config.display()
        self.model = modellib.MaskRCNN(mode="inference", config=self.config, model_dir=MODEL_DIR)        
        self.model_path = self.model.find_last()    
        
        assert self.model_path != "", "Provide path to trained weights"
        print("Loading weights from ", self.model_path)
        self.model.load_weights(self.model_path, by_name=True)     
        
        self.class_names = ['BG', 'head']   

        self.results = self.model.detect([self.image])
        r = self.results[0]
        masks = r['masks']
        mask = masks[:, :, 0]                     

        self.total_point = np.sum(mask) 
        self.point_coord = np.empty((2,self.total_point))
        mask = np.flipud(mask)
        
        self.point_coord = get_app_index(mask,self.point_coord) 
        self.arr = self.point_coord
        
        self.open = 1
        
        visualize.display_instances(self.image, r['rois'], r['masks'], r['class_ids'], self.class_names,r['scores'])        


    def callback_point(self, msg):
        if self.open == 0:
            pass
        elif self.open == 1:
            self.points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"))         

            self.mask_coord = np.empty((3,self.total_point))                       

            mask_coord, point_centroid  = get_app_coord(self.arr,self.total_point,self.mask_coord,self.points)
            
            T_Cam_l,T_ECM_g = get_ECM()
            T_psml_g = get_PSM_l()
            
            point_centroid.append(1)
            
            app_head_psml = get_app_head_psml(T_Cam_l,T_ECM_g,T_psml_g,point_centroid)
            
            self.point_centroid = np.array(point_centroid, dtype=np.float32)
            self.app_head_psml = np.array(app_head_psml, dtype=np.float32)

            
            
            self.pub.publish(self.app_head_psml)
            
            print('\n','point_centroid: ',self.point_centroid,'\n')
            print('\n','app_head_psml: ',self.app_head_psml,'\n')
            
            self.open = 0
        
def run_loop():
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        rate.sleep() 
        
if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    node = PointCloudSubscriber()
    run_loop()                                    
            
                    