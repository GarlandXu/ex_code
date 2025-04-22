import sys
sys.path.append("/home/haoyuan/catkin_ws/src/appendectomy")

from utilities import *
from psms.psmL import psmL
from psms.psmR import psmR
from ambf_client import Client
from agent.GUI_v1 import GUI
import rospy
from ik.traj_gen import Traj_gen
from DMP.dmp import dmp_discrete
from lfd.DMP.cartesian_dmp import DMP
import json
import threading
import tkinter as tk
import time
import pickle



class Peg_transfer_controller:
    def __init__(self):
        self.client = client

        # get the PSMs class
        self.left_psm = psmL(client=self.client)
        self.right_psm = psmR(client=self.client)

        # get the appendix class
        self.root = self.client.get_obj_handle('Root')
        self.middle_1 = self.client.get_obj_handle('Middle_1')
        self.middle_2 = self.client.get_obj_handle('Middle_2')
        self.middle_6 = self.client.get_obj_handle('Middle_6')
        self.head = self.client.get_obj_handle('Head')
        
        # scene registration
        self.scene_file = f'scene.json'
        with open(self.scene_file, 'r') as file:
            self.scene = json.load(file)

        # save the JSON name and handle name
        self.obj_names = {
                    "left_psm": self.left_psm,
                    "right_psm": self.right_psm,
                    "head": self.head,
                    "middle_1": self.middle_1,
                    "middle_2": self.middle_2,
                    "middle_6": self.middle_6,
                    "root": self.middle_1
                    }     

        # DMP
        grasp_file = f'grasp.pkl'
        lift_file = f'lift.pkl'
        obs_file = f'obs.pkl'

        self.traj_gen = Traj_gen()
        self.dmp_grasping = read_pickle(grasp_file)
        self.dmp_lift = read_pickle(lift_file)
        self.dmp_obs = read_pickle(lift_file)

        self.paused_event = threading.Event()
        self.paused_event.set()

        self.terminate_thread = False  # Initialize the termination flag
        self.execution_thread = None

        # Agent
        self.gui = GUI()


    def locate_obj_pose(self, json_name):
        pos = self.scene[json_name]['position']
        rot = self.scene[json_name]['orientation']
        pose = np.concatenate((pos, rot))
        return pose
    
    # pos: read out angle and use FK to get the position
    # angle: be careful with different tools
    def update_scene(self):
        for json_name, handle in self.obj_names.items():
            if handle == self.left_psm or handle == self.right_psm:
                pose = handle.get_pos()
                gripper_angle = handle.get_tip_angle()
                if abs(gripper_angle) < 0.2:
                     self.scene[json_name]['jaw_status'] = "closed"
                else:
                    self.scene[json_name]['jaw_status'] = "open"

            else:
                pos = handle.get_pos()
                rot = handle.get_rot()

                pose = np.array([pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w])
            
            # print(json_name, pose)

            self.scene[json_name]['position'] = [pose[0], pose[1], pose[2]]
            self.scene[json_name]['orientation'] = [pose[3], pose[4], pose[5], pose[6]]
        
        print("Scene has been updated!")    



    #  be sure abt the reasoning patr: into the mid-air and what?
    def update_planner(self):
        self.update_scene()

        left_pose = str(self.scene["left_psm"]['position']) + str(self.scene["left_psm"]['orientation'])
        left_status = str(self.scene["left_psm"]['jaw_status'])

        right_pose = str(self.scene["right_psm"]['position']) + str(self.scene["right_psm"]['orientation'])
        right_status = str(self.scene["right_psm"]['jaw_status'])

        root_pose = str(self.scene["root"]['position']) + str(self.scene["root"]['orientation'])
        middle_1_pose = str(self.scene["root"]['position']) + str(self.scene["root"]['orientation'])
        middle_2_pose = str(self.scene["middle_2"]['position']) + str(self.scene["middle_2"]['orientation'])
        middle_6_pose = str(self.scene["middle_6"]['position']) + str(self.scene["middle_6"]['orientation'])
        head_pose = str(self.scene["head"]['position']) + str(self.scene["head"]['orientation'])



        logbook = "\nNow the left PSM position is: " + left_pose + " and its jaw is " + left_status\
                  +"\n the right PSM position is: " + right_pose + " and its jaw is " + right_status\
                  +"\n root pose is: " + root_pose\
                  +"\n middle_1 pose is: " +  middle_1_pose\
                  +"\n middle_2 pose is: " + middle_2_pose\
                  +"\n middle_6 pose is: " + middle_6_pose\
                  +"\n head pose is: " + head_pose
        
        # print("\n\nThe observed updates from the scene is shown as follows :", logbook)
        response = self.gui.planner.get_response(logbook) 
        # print("\nresponse after logbook:", response)
        # self.gui.display_response(response)
        print("Planner has been updated!")    
    
    def which_gripper(self, psm_name):
        if psm_name == "left_psm":
            psm = self.left_psm
        elif psm_name == "right_psm":
            psm = self.right_psm  
        return psm
    
    # need to set up the logic inside the class
    def close_jaw(self, psm_name):
        psm = self.which_gripper(psm_name)
        psm.set_tip_angle(-0.1)
        time.sleep(1)
        
    def open_jaw(self, psm_name):
        psm = self.which_gripper(psm_name)
        psm.set_tip_angle(-0.35)
        time.sleep(1)

    # def approach
    def approach(self, goal_pos, psm_name):
        # get pos for obj and gripper 
        psm_pos = self.locate_obj_pose(psm_name)
        # print(gripper_pos)
        # print(goal_pos)
        psm = self.which_gripper(psm_name)
        traj_reproduce = self.dmp_grasping.reproduce(initial=psm_pos, goal=goal_pos)
        # print(traj_reproduce.shape)
        # print("FOr", gripper_pos)
        # print("downing", traj_reproduce[:,-1])
        # reproduce learned trajectory
        self.traj_gen.set_traj(traj_reproduce)
        psm.exeutce_pose_trajectory(self.traj_gen, 5, 120, self)
        # psm.exec_traj(self.traj_gen, 20, 50)
        self.update_scene()
        time.sleep(0.1)

    def lift(self, goal_pos, psm_name):
        # get pos for obj and gripper 
        psm_pos = self.locate_obj_pose(psm_name)
        # print(gripper_pos)
        # print(goal_pos)
        psm = self.which_gripper(psm_name)
        traj_reproduce = self.dmp_grasping.reproduce(initial=psm_pos, goal=goal_pos)
        # print(traj_reproduce.shape)
        # print("FOr", gripper_pos)
        # print("downing", traj_reproduce[:,-1])
        # reproduce learned trajectory
        self.traj_gen.set_traj(traj_reproduce)
        psm.execute_pose_trajectory(self.traj_gen, 5, 120, self)
        # psm.exec_traj(self.traj_gen, 20, 50)
        self.update_scene()
        time.sleep(0.1)

    def stretch(self, psm_name):
        self.update_scene()
        offset = np.array([0, 0.12, 0, 0, 0, 0, 0])
        psm_pos = self.locate_obj_pose(psm_name)
        psm = self.which_gripper(psm_name)
        goal_pos = [a + b for a, b in zip(offset, psm_pos)]
        

        traj_reproduce = self.dmp_grasping.reproduce(initial=psm_pos, goal=goal_pos)
        self.traj_gen.set_traj(traj_reproduce)
        psm.execute_pose_trajectory(self.traj_gen, 5, 120, self)
        # psm.exec_traj(self.traj_gen, 20, 50)
        self.update_scene()
        time.sleep(0.1)


    def probing(self, psm_name):
        self.update_scene()
        psm_pos = self.locate_obj_pose(psm_name)
        psm = self.which_gripper(psm_name)

        # Extract position and quaternion
        x, y, z, qx, qy, qz, qw = psm_pos
        
        # Create a quaternion for the z-axis rotation (50 degrees in radians)
        angle_deg = -60
        angle_rad = math.radians(angle_deg)
        rotation_z = R.from_euler('y', angle_rad, degrees=False)
        
        # Create a quaternion from the original pose
        original_quaternion = R.from_quat([qx, qy, qz, qw])
        
        # Combine the rotations (z-axis rotation followed by the original rotation)
        new_quaternion = rotation_z * original_quaternion
        
        # Extract the new quaternion values
        new_qx, new_qy, new_qz, new_qw = new_quaternion.as_quat()
        
        # New pose with the rotated orientation
        goal_pos = np.array([x, y, z, new_qx, new_qy, new_qz, new_qw])


        traj_reproduce = self.dmp_grasping.reproduce(initial=psm_pos, goal=goal_pos)
        self.traj_gen.set_traj(traj_reproduce)
        psm.execute_pose_trajectory(self.traj_gen, 5, 50, self)
        # psm.exec_traj(self.traj_gen, 20, 50)
        self.update_scene()
        time.sleep(0.1)




def get_response(peg_con):
    peg_con.update_scene()
    command = peg_con.gui.prompt_entry.get("1.0", tk.END).strip()
    if command:
        peg_con.gui.planner_response = peg_con.gui.planner.get_response(command)
        peg_con.gui.display_response(peg_con.gui.planner_response)
        peg_con.gui.prompt_entry.delete("1.0", tk.END)
    else:
        peg_con.gui.display_response("Please enter a command.")

def pause_event(peg_con):
   peg_con.paused_event.clear()
   peg_con.update_planner()

def continue_event(peg_con):
    peg_con.paused_event.set()
    # peg_con.update_scene()

def save_and_execute(peg_con):
    def exec_fcn():
        plan = peg_con.gui.planner_response
        reminder = str("\n\n Please make sure your output function can be excuted directly in the terminal. No comments or explanation needed, just the code.")
        plan = plan + reminder
        print(plan)
        try:
            functions = peg_con.gui.get_function(plan)
            print("coder:", functions)
            lines = functions.split('\n')
            for line in lines:
                while not peg_con.paused_event.is_set():
                    # Wait briefly to allow frequent checks for terminate_thread flag
                    peg_con.paused_event.wait(timeout=0.1)
                    if peg_con.terminate_thread:
                        return  # Clean exit if termination is signaled
                exec(line, globals())
                if peg_con.terminate_thread:
                    return  # Also check for termination after executing a line
        except Exception as e:
            peg_con.gui.display_response("Please restart the system!")

    # Interrupt the old thread if it is still running
    if getattr(peg_con, 'execution_thread', None) and peg_con.execution_thread.is_alive():
        peg_con.terminate_thread = True
        peg_con.execution_thread.join(timeout=2)  # Timeout to avoid hanging
        if peg_con.execution_thread.is_alive():
            peg_con.gui.display_response("Failed to terminate old thread. Please retry.")

    # Reset the flag and start a new thread
    peg_con.terminate_thread = False
    peg_con.paused_event.set()  # Ensure it starts in an unpaused state
    peg_con.execution_thread = threading.Thread(target=exec_fcn)
    peg_con.execution_thread.start()

def config_app(gui):
    gui.continue_button.config(command=lambda: continue_event(peg_con))
    gui.paused_button.config(command=lambda: pause_event(peg_con))
    gui.action_button.config(command=lambda: save_and_execute(peg_con))
    gui.submit_button.config(command=lambda: get_response(peg_con))

 
if __name__ == '__main__':

    client = Client()
    client.connect()
    rospy.Rate(50)

    peg_con = Peg_transfer_controller()

    config_app(peg_con.gui)
    peg_con.gui.run()


   