from ambf_client import Client
import time
import rospy

import sys
sys.path.append("/home/haoyuan/catkin_ws/src/appendectomy")
from teleo.omni_device import Omni_device
from psms.psmL import psmL
from psms.psmR import psmR
from teleo_utilities import *
from pynput.keyboard import Key, Listener
import numpy as np
from threading import Thread


class gripper_controller:
    def __init__(self, omni_L, left_gripper, omni_R, right_gripper, demoed):
        # set AMBF client 
        self.client = client

        # Omni device
        self.omni_L = omni_L
        self.omni_R = omni_R

        self.handled = False

        self.left_gripper = left_gripper
        self.right_gripper = right_gripper

        self.scaling_coeff = 0.005

        # right psm
        right_pos_i = self.omni_R.get_pose().p
        self.right_handled = False
        self.right_pos_i = np.array([right_pos_i[0], right_pos_i[1], right_pos_i[2]])
        right_gripper_pos = self.right_gripper.get_pos()
        self.right_gripper_pos = np.array([right_gripper_pos[0],right_gripper_pos[1],right_gripper_pos[2]])

        # left psms
        left_pos_i = self.omni_L.get_pose().p
        self.left_handled = False
        self.left_pos_i = np.array([left_pos_i[0], left_pos_i[1], left_pos_i[2]])
        left_gripper_pos = self.left_gripper.get_pos()
        self.left_gripper_pos = np.array([left_gripper_pos[0],left_gripper_pos[1],left_gripper_pos[2]])
        # print(self.left_gripper_pos)

        # self.peg_1 = self.client.get_obj_handle("PuzzleRed1")
        # self.col_1 = self.client.get_obj_handle("Cylinder_002")

        self.demo_folder = f'/home/haoyuan/catkin_ws/src/appendectomy/lfd/data'
        if demoed:
             self.demo_file = get_next_demo_file(self.demo_folder)

        time.sleep(0.5)
        print("initialized!")

             

            
    def teleo_left(self):
        self.left_quan = self.omni_L.get_pose().M .GetQuaternion()
        # print(self.left_quan)

        left_pos = self.omni_L.get_pose().p
        self.left_pos = np.array([left_pos[0], left_pos[1], left_pos[2]])
        
        if self.omni_L.clutch_button_pressed and not self.left_handled:
              left_gripper_pos = self.left_gripper.get_pos()
              self.left_gripper_pos = np.array([left_gripper_pos[0],left_gripper_pos[1],left_gripper_pos[2]])
            #   print(self.left_gripper_pos)
              self.left_handled = True
        elif self.omni_L.clutch_button_pressed and self.left_handled:
              left_pos = self.omni_L.get_pose().p
              self.left_pos_i = np.array([left_pos[0], left_pos[1], left_pos[2]])
            #   self.right_pos_i = self.right_pos
        elif not self.omni_L.clutch_button_pressed:
             self.left_handled = False

        self.left_diff = self.left_pos - self.left_pos_i
        self.left_trans = self.left_gripper_pos + self.left_diff * self.scaling_coeff
        # print(self.left_trans)
        # sets coord
        self.left_trans = np.concatenate((self.left_trans, self.left_quan))
        # print(self.left_trans)
        self.left_gripper.set_pos(self.left_trans)
        # self.left_gripper.set_rpy(self.left_rpy)
        self.left_gripper.set_tip_angle(self.omni_L.get_gripper_angle())   


    def teleo_right(self):
        self.right_quan = self.omni_R.get_pose().M .GetQuaternion()

        right_pos = self.omni_R.get_pose().p
        self.right_pos = np.array([right_pos[0], right_pos[1], right_pos[2]])
        
        if self.omni_R.clutch_button_pressed and not self.right_handled:
              right_gripper_pos = self.right_gripper.get_pos() 
              self.right_gripper_pos = np.array([right_gripper_pos[0],right_gripper_pos[1],right_gripper_pos[2]])
            #   self.right_pos_i = self.right_pos
              self.right_handled = True
              
        elif self.omni_R.clutch_button_pressed and self.right_handled:
              righ_pos = self.omni_R.get_pose().p
              self.right_pos_i = np.array([righ_pos[0], righ_pos[1], righ_pos[2]])

        elif not self.omni_R.clutch_button_pressed:
             self.right_handled = False   
            
        self.right_diff = self.right_pos - self.right_pos_i
        self.right_trans = self.right_gripper_pos + self.right_diff * self.scaling_coeff
        self.right_trans = np.concatenate((self.right_trans, self.right_quan))
        self.right_gripper.set_pos(self.right_trans)
        self.right_gripper.set_tip_angle(self.omni_R.get_gripper_angle())   
        
        # sets coord
        # cool = self.right_gripper.get_pos()
        # x = self.peg_1.get_pos()
        # x = ambf_pos_to_arr(x)

        # x1 = self.col_1.get_pos()
        # x1 = ambf_pos_to_arr(x1)
        # print("peg position is:", x)
        # print("col1 pos is:",x1)
        # print("The gripper pos is: ", cool)
        # offset = cool - x1
        # print("offset:",offset)
        # self.right_gripper.set_rpy(self.right_rpy)
        # self.right_gripper.set_jaw_angle(self.omni_R.get_gripper_angle())    


    def update_arm_pose(self):
        t1 = Thread(target=self.teleo_left)
        t2 = Thread(target=self.teleo_right)
        t1.start()
        t2.start()
        t1.join()
        t2.join()

    def record_state(self):
           self.left_gripper.state = self.omni_L.record_state
           self.right_gripper.state = self.omni_R.record_state    

    def run(self):
        self.record_state()
        if self.omni_R.recording :
           self.update_arm_pose()
        # does it has collision?
        if self.omni_L.recording :
             record_data(self.demo_file, self.left_gripper, self.right_gripper)
        # if self.omni_R.recording :
        #      record_data(self.demo_file, self.left_gripper, self.right_gripper) 

        # off_1 = [-0.15,0.06, 0.49]
        # off_2 = [-0.119, 0.08, 0.4884]

        # x = self.peg_1.get_pos()
        # x = ambf_pos_to_arr(x)

        # x1 = self.col_1.get_pos()
        # x1 = ambf_pos_to_arr(x1)
        
        # pos = off_1 + x
        # self.right_gripper.set_pos(pos)   
           
        
if __name__ == "__main__":
    client = Client()
    client.connect()
    controllers = []
    rate = rospy.Rate(50)
    time.sleep(0.3)
    
    omni_R = Omni_device('/Geomagic_R/')
    omni_L = Omni_device('/Geomagic_L/')
    
    left_psm = psmL(client)
    right_psm = psmR(client)
    

    cont = gripper_controller(omni_L, right_psm, omni_R, left_psm, demoed = True)    
    time.sleep(1)
    print("Teleoperation Begin!")                                                                                                                                                                                         
    
    try:
            while not rospy.is_shutdown():
                cont.run()
                rate.sleep()
               
    except:
            print('WTF!!!!!')


        

