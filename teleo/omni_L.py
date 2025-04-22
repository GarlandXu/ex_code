#!/usr/bin/env python

import PyKDL
from PyKDL import Frame, Rotation, Vector
from geomagic_control.msg import DeviceButtonEvent
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
import math
from teleo_utilities import *
import sys
sys.path.append("/home/haoyuan/catkin_ws/src/appendectomy")
from utilities import *
from std_msgs.msg import Float32, Float32MultiArray, Bool


def world_to_psm2(world):
    world = np.array(world)
    pose_world = pose_from_traj(world)
    # transform array based on:
    # t.transform.translation.x = self.PSM2_traj[2] 
    # t.transform.translation.y = self.PSM2_traj[0] 
    # t.transform.translation.z = self.PSM2_traj[1] 
    # T = np.array([
    #     [0, 0, 1, 0],
    #     [1, 0, 0, 0],
    #     [0, 1, 0, 0],
    #     [0, 0, 0, 1]
    # ])

    T = np.array([
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1]
    ])

    # T = np.eye(4)

    world_in_psm2_pose = np.dot(T , pose_world)
    traj_world_in_psm2 = traj_from_pose(world_in_psm2_pose)

    return traj_world_in_psm2



class omni_L:
    def __init__(self, name):
        pose_topic_name = name + 'pose'
        button_topic_name = name + 'button'

        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        left_pos_i = self.pose.p
        self.left_pos_i = np.array([left_pos_i[0], left_pos_i[1], left_pos_i[2]])
        self.left_handled = False
        self._scale = 0.0002

        # base and tip frame in the simulation
        R_off = Rotation.RPY(-math.pi / 2.0, -math.pi / 2.0, 0)
        self._T_baseoffset = Frame(R_off, Vector(0, 0, 0))
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        self._T_tipoffset = Frame(Rotation().RPY(0, -math.pi / 2.0,  -math.pi / 2.0,), Vector(0, 0, 0))

        self.clutch_button_pressed = False  
        self.gripper_button_pressed = False  
        self.gripper_angle = 0.6

        self.update_pos = Bool()
        self.update_pos.data = False

        # publisher and subsrciber
        self._xyz_pub = rospy.Publisher('/trans_pos_omni_L', Float32MultiArray, queue_size=10)
        self._pos_pub = rospy.Publisher('/pos_omni_L', Float32MultiArray, queue_size=10)
        self._jaw_pub = rospy.Publisher('/jaw_omni_L', Float32, queue_size=10)
        self._update_pub = rospy.Publisher('/pos_update_omni_L', Bool, queue_size=10)
        self._clutch_pub = rospy.Publisher('/clucthed_omni_L', Bool, queue_size=10)

        self._pose_sub = rospy.Subscriber(
            pose_topic_name, PoseStamped, self.pose_cb, queue_size=10)
        self._button_sub = rospy.Subscriber(
            button_topic_name, DeviceButtonEvent, self.button_cb, queue_size=1)


        print('Creating Geomagic Device Named: ', name, ' From ROS Topics')

    def set_base_frame(self, frame):
        self._T_baseoffset = frame
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        pass

    def set_tip_frame(self, frame):
        self._T_tipoffset = frame
        pass

    # detect button events    
    def button_cb(self, msg):
        self.gripper_button_pressed = msg.white_button
        self.clutch_button_pressed = msg.grey_button
        
        # clutch
        if self.clutch_button_pressed:
            print('clutched!')
            
        # get gripper angle    
        if self.gripper_button_pressed:
            self.gripper_angle = -0.1
        else:
            self.gripper_angle = 0.6

    # set the pose 
    def pose_cb(self, msg):
        cur_frame = pose_msg_to_kdl_frame(msg)
        self.pose = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset #teleoperation pose

        left_pos = self.pose.p
        self.left_pos = np.array([left_pos[0], left_pos[1], left_pos[2]])
        self.quan = self.pose.M.GetQuaternion()  # Returns (x, y, z, w)
        omni_traj = np.concatenate([self.left_pos,self.quan]) 
        
        if self.clutch_button_pressed and not self.left_handled:
            #   self.left_gripper_pos = self.left_gripper.get_pos()
              self.update_pos = True
              self._update_pub.publish(self.update_pos)
              self.left_handled = True
        elif self.clutch_button_pressed and self.left_handled:
              left_pos = self.pose.p
              self.left_pos_i = np.array([left_pos[0], left_pos[1], left_pos[2]])
            #   self.right_pos_i = self.right_pos
        elif not self.clutch_button_pressed:
             self.left_handled = False

        self.left_diff = (self.left_pos - self.left_pos_i) * self._scale
        traj = np.concatenate([self.left_diff, self.quan])
        dvrk_traj = world_to_psm2(traj)


        round = np.round(dvrk_traj, decimals = 6)

        # print("traj in world", traj)
        # print("traj in dvrk", dvrk_traj)
        
        
        dvrk_xyz = Float32MultiArray()
        dvrk_xyz.data = round.tolist()

        # print("sent trans:", dvrk_xyz)
        # print("pos",left_pos)

        xyz = Float32MultiArray()
        xyz.data = omni_traj.tolist()
        # rospy.loginfo(f"Publishing XYZ: {self.left_pos}")
        self._pos_pub.publish(xyz)

        # rospy.loginfo(f"Publishing XYZ: {dvrk_xyz.data}")
        self._xyz_pub.publish(dvrk_xyz)

        # rospy.loginfo(f"Publishing JAW: {self.}")    
        self._jaw_pub.publish(self.gripper_angle)

        k = Bool()
        k.data = self.clutch_button_pressed
        self._clutch_pub.publish(k)
    
    

def run():
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('omni_L')
    # left = omni_L('/Geomagic_L/')
    left = omni_L('/Geomagic_L/')
    run()
