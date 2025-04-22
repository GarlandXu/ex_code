#!/usr/bin/env python

import PyKDL
from PyKDL import Frame, Rotation, Vector
from geomagic_control.msg import DeviceButtonEvent
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
import math
from teleo_utilities import pose_msg_to_kdl_frame
from std_msgs.msg import Float32, Float32MultiArray, Bool
import sys
sys.path.append("/home/haoyuan/catkin_ws/src/appendectomy")
from utilities import *


# transformation between PSM1 and PSM2
# NEED TO CHECK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def psm1_to_psm2(psm1):
    trans_matrix = calculate_transform([-0.066244, -0.14368, 0.053877], [-0.060038, -0.14417, 0.076071],  
    [0.087233, -0.152899, -0.016296],  [0.086153, -0.152717, 0.00707])
    psm1 = np.array(psm1)
    pose_psm1 = pose_from_traj(psm1)
    pose_psm1_in_psm2 = np.dot(trans_matrix, pose_psm1)
    traj_psm1_in_psm2 = traj_from_pose(pose_psm1_in_psm2)

    return traj_psm1_in_psm2


# the trans matrix should be inversed, as we alway input the output from calculate_transform
def psm2_to_psm1(psm2):
    trans_matrix = calculate_transform([-0.066244, -0.14368, 0.053877], [-0.060038, -0.14417, 0.076071],  
    [0.087233, -0.152899, -0.016296],  [0.086153, -0.152717, 0.00707])

    psm2 = np.array(psm2)
    pose_psm2 = pose_from_traj(psm2)
    pose_psm2_in_psm1 = np.dot(np.linalg.inv(trans_matrix),pose_psm2) 
    traj_psm2_in_psm1 = traj_from_pose(pose_psm2_in_psm1)

    return traj_psm2_in_psm1


def psm2_to_world(psm2):
    psm2 = np.array(psm2)
    psm2 = pose_from_traj(psm2)

    T_inv = np.array([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    # T_inv = np.array([
    #     [0, 1, 0, 0],
    #     [0, 0, 1, 0],
    #     [1, 0, 0, 0],
    #     [0, 0, 0, 1]
    # ])

    # T_inv = np.eye(4)


    pose_psm2_in_world = np.dot(T_inv , psm2)
    traj_psm2_in_world = traj_from_pose(pose_psm2_in_world)

    return traj_psm2_in_world


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


def psm1_to_world(psm1):
    psm1 = np.array(psm1)
    traj_psm1_in_psm2 = psm1_to_psm2(psm1)
    traj_psm1_in_world = psm2_to_world(traj_psm1_in_psm2)

    return traj_psm1_in_world


def world_to_psm1(world):
    world = np.array(world)
    world_in_psm2 = world_to_psm2(world)
    world_in_psm1 = psm2_to_psm1(world_in_psm2)
   
    return world_in_psm1


class omni_R:
    def __init__(self, name):
        pose_topic_name = name + 'pose'
        button_topic_name = name + 'button'

        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        right_pos_i = self.pose.p
        self.right_pos_i = np.array([right_pos_i[0], right_pos_i[1], right_pos_i[2]])
        self.right_handled = False
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
        self._clutch_pub = rospy.Publisher('/clucthed_omni_R', Bool, queue_size=10)
        self._xyz_pub = rospy.Publisher('/trans_pos_omni_R', Float32MultiArray, queue_size=10)
        self._pos_pub = rospy.Publisher('/pos_omni_R', Float32MultiArray, queue_size=10)
        self._jaw_pub = rospy.Publisher('/jaw_omni_R', Float32, queue_size=10)
        self._update_pub = rospy.Publisher('/pos_update_omni_R', Bool, queue_size=10)

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
            self.gripper_angle = 0.7
            print("Right PSM closed!")
        else:
            self.gripper_angle = 0.7

    # set the pose 
    def pose_cb(self, msg):
        cur_frame = pose_msg_to_kdl_frame(msg)
        self.pose = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset #teleoperation pose

        right_pos = self.pose.p
        self.right_pos = np.array([right_pos[0], right_pos[1], right_pos[2]])
        self.quan = self.pose.M.GetQuaternion()  # Returns (x, y, z, w)

        omni_traj = np.concatenate([self.right_pos,self.quan]) 
        
        if self.clutch_button_pressed and not self.right_handled:
            #   self.right_gripper_pos = self.right_gripper.get_pos()
              self.update_pos = True
              self.right_handled = True
        elif self.clutch_button_pressed and self.right_handled:
              right_pos = self.pose.p
              self._update_pub.publish(self.update_pos)
              self.right_pos_i = np.array([right_pos[0], right_pos[1], right_pos[2]])
            #   self.right_pos_i = self.right_pos
        elif not self.clutch_button_pressed:
             self.right_handled = False
             self.update_pos = False
            

        self.right_diff = (self.right_pos - self.right_pos_i) * self._scale
        traj = np.concatenate([self.right_diff, self.quan])

        # slight rotation between psm1 and psm2,huge in tranlation.
        dvrk_traj = world_to_psm2(traj)
        # print(dvrk_traj)

        # print(dvrk_traj)
        round = np.round(dvrk_traj, decimals = 6)
        
        dvrk_xyz = Float32MultiArray()
        dvrk_xyz.data = round.tolist()

        xyz = Float32MultiArray()
        xyz.data = omni_traj.tolist()

        # print("sent trans:", dvrk_xyz)
        # print("pos",right_pos)
        k = Bool()
        k.data = self.clutch_button_pressed
         # rospy.loginfo(f"Publishing XYZ: {self.left_pos}")
        self._pos_pub.publish(xyz)
        # rospy.loginfo(f"Publishing XYZ: {dvrk_xyz.data}")
        self._xyz_pub.publish(dvrk_xyz)

        # rospy.loginfo(f"Publishing JAW: {self.gripper_angle}")    
        self._jaw_pub.publish(self.gripper_angle)
        self._clutch_pub.publish(k)
    
    

def run():
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('omni_R')
    right = omni_R('/Geomagic_R/')
    run()
