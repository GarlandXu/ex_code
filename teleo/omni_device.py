#!/usr/bin/env python

import PyKDL
from PyKDL import Frame, Rotation, Vector
from geomagic_control.msg import DeviceFeedback, DeviceButtonEvent
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy,JointState
import rospy
import time
import numpy as np
import math
from teleo_utilities import *
import threading

class Omni_device:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name):
        pose_topic_name = name + 'pose'
        twist_topic_name = name + 'twist'
        button_topic_name = name + 'button'
        joint_topic_name = name + 'joint_states'

        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.twist = PyKDL.Twist()
        self._scale = 0.02

        # base and tip frame in the simulation
        R_off = Rotation.RPY(-math.pi / 2.0, -math.pi / 2.0, 0)
        self._T_baseoffset = Frame(R_off, Vector(0, 0, 0))
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        self._T_tipoffset = Frame(Rotation().RPY(0, -math.pi / 2.0,  -math.pi / 2.0,), Vector(0, 0, 0))

        self.clutch_button_pressed = False  
        self.gripper_button_pressed = False  
        self.gripper_angle = -0.35
        self.ref_pos = 0
        self.quan = [0, 0, 0, 0]
        self.RPY = [0,0,0]
        self.saved_omni_pos = 0
        self.record_state = 0

        #publisher and subsrciber
        self._pose_sub = rospy.Subscriber(
            pose_topic_name, PoseStamped, self.pose_cb, queue_size=10)
        self._twist_sub = rospy.Subscriber(
            twist_topic_name, Twist, self.twist_cb, queue_size=1)
        self._button_sub = rospy.Subscriber(
            button_topic_name, DeviceButtonEvent, self.button_cb, queue_size=1)
        self._joint_sub = rospy.Subscriber(
            joint_topic_name, JointState, self.joint_cb)
        
        self.recording = False
        self.recording_time = rospy.Duration(0.5)
        self._button_msg_time = rospy.Time.now()
        print('Creating Geomagic Device Named: ', name, ' From ROS Topics')
        self._msg_counter = 0

    def set_base_frame(self, frame):
        self._T_baseoffset = frame
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        pass

    def set_tip_frame(self, frame):
        self._T_tipoffset = frame
        pass

     # get velocities
    def twist_cb(self, msg):
        twist = PyKDL.Twist()
        twist[0] = msg.linear.x
        twist[1] = msg.linear.y
        twist[2] = msg.linear.z
        twist[3] = msg.angular.x
        twist[4] = msg.angular.y
        twist[5] = msg.angular.z
        self.twist = self._T_baseoffset_inverse * twist

    # get the joint angles of omni    
    def joint_cb(self, msg):
        self.omni_joint = msg.position 
        #print(msg.position)

    # detect button events    
    def button_cb(self, msg):
        self.gripper_button_pressed = msg.white_button
        self.clutch_button_pressed = msg.grey_button
        
        # clutch
        if self.clutch_button_pressed:
            print('clutched!')
            time_diff = rospy.Time.now() - self._button_msg_time
            # if clutched twice
            if time_diff.to_sec() < self.recording_time.to_sec() and self.recording == False:
                print('Start Recording!')
                self.recording = True
                self.record_state += 1
            elif time_diff.to_sec() < self.recording_time.to_sec() and self.recording == True:
                print('End Recording!')
                self.recording = False
            
            self._button_msg_time = rospy.Time.now()
            
        # get gripper angle    
        if self.gripper_button_pressed:
            self.gripper_angle = -0.1
        else:
            self.gripper_angle = -0.35

    # set the pose 
    def pose_cb(self, msg):
        cur_frame = pose_msg_to_kdl_frame(msg)
        self.pose = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset #teleoperation pose
        
    def get_pose(self):
        return self.pose
    
    def get_twist(self):
        return self.twist
    
    def get_gripper_angle(self):
        return self.gripper_angle
    
    

def test():
    rospy.init_node('test_geomagic')

    d = Omni_device('/Geomagic_L/')

    while not rospy.is_shutdown():
        h = d.get_gripper_angle()
        time.sleep(0.05)


if __name__ == '__main__':
    test()
