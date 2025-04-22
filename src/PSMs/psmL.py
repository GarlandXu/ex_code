import rospy
import time
import numpy as np
import math
from ambf_client import Client
from threading import Thread, Lock

Your_path




class psmL:
    def __init__(self, client):
        self._client = client

        # get handles for left PSM
        self._handle_left = self._client.get_obj_handle('baselink_l')
        self.sensor = self._client.get_obj_handle('Proximity_l')
        self.actuator = self._client.get_obj_handle('Constraint_l1')
        self.actuator_1 = self._client.get_obj_handle('Constraint_l2')  
        self.actuator_2 = self._client.get_obj_handle('Constraint_l3')
        self.actuators = [self.actuator, self.actuator_1, self.actuator_2]
        time.sleep(0.1)

        # initial settings for left PSMs
        inital_joint_1 = 0.5259690284729004
        inital_joint_2 = -0.47849902510643005
        inital_joint_3 = 1.8876142501831055 
        self.q_5 = 0
        self.q_tip = -0.35 # tip angle
        time.sleep(0.5)

        # set initial pos
        set_inital_L_pos(self._handle_left,inital_joint_1,inital_joint_2,inital_joint_3)
        self.set_tip_angle(self.q_tip)
        self.pos = self.get_pos()
        
        # fcn to get back the quan
        self.quan = self.get_quan()
        self.grasped = False

        # thread initializtation
        self._thread_lock = Lock()
        self._force_exit_thread = False

        # need to put initial pos and quan
        self.recorded_traj = [0,0,0]

        # need to figure out the joint limits
        # self.joint_limits = [[-0.3, 0.548], [-0.3, 0.548], [-0.3, 0.548], [-0.3, 0.548]]

        # self.set_pos(self.initial_pos)
        # self.set_rpy(self.rpy)
        self.set_tip_angle(self.q_tip)
    

# x,y,z,qx,qy,qz,qw
# for the pose we only care abt the link, not the gripper, as they are fixed
# pose should be set in world frame!!!
    def set_pos(self, traj_in_world):
        # mappping
        traj_in_psml = obj_in_PSM_l(traj_in_world)
        tran_matrix = pose_from_traj(traj_in_psml)

        # IK  
        computed_q = compute_IK(convert_mat_to_frame(tran_matrix))
        # print("q",computed_q)

        self._handle_left.set_joint_pos('baselink-yawlink_l', computed_q[0])
        self._handle_left.set_joint_pos('yawlink-pitchbacklink_l', computed_q[1])
        self._handle_left.set_joint_pos('pitchendlink-maininsertionlink_l', computed_q[2])
        self._handle_left.set_joint_pos('maininsertionlink-toolrolllink_l',computed_q[3])
        self._handle_left.set_joint_pos('toolrolllink-toolpitchlink_l',computed_q[4])
        self.q_5 = computed_q[5]
        # self._handle_left.set_joint_pos('toolpitchlink-toolgripper1link_l',computed_q[5])
        # self._handle_left.set_joint_pos('toolpitchlink-toolgripper2link_l',computed_q[5])
        # print(computed_q[5])
        self.pos = self.get_pos() 
        self.set_tip_angle(self.q_tip)
        # FK
        # self.pos = self.get_pos() 

    
    def get_pos(self):
       # FK
        self.joints = get_PSML_pos(self._handle_left)
        self.joints[5] = self.q_5
        self.joints[6] = self.q_5
        pose_map = round_mat(compute_FK(self.joints),4,4,3)

        pose_in_psml = traj_from_pose(pose_map)
        self.pos = psml_in_world(pose_in_psml)
        
        return self.pos
        

    def get_quan(self):
        quan = self.gripper_base.get_rot()
        self.quan[0] = quan.x
        self.quan[1] = quan.y
        self.quan[2] = quan.z
        self.quan[3] = quan.w
        return self.quan
    
    def get_tip_angle(self):
        return self.q_tip

    def set_tip_angle(self, tip_angle):
        self.q_tip = tip_angle
        self._handle_left.set_joint_pos('toolpitchlink-toolgripper1link_l', self.q_tip)
        self._handle_left.set_joint_pos('toolpitchlink-toolgripper2link_l', self.q_tip)
        
        for i in range(self.gripper_base.get_num_joints()):
            jnt_range = self.joint_limits[i][1] - self.joint_limits[i][0]
            adjusted_val = self.joint_limits[i][0] + gripper_angle * jnt_range
            self.gripper_base.set_joint_pos(i, adjusted_val)
        # grasping_logic
        self.run_grasp_logic()


    def run_grasp_logic(self):
        if self.sensor.is_triggered(0) and abs(self.q_tip) < 0.2:
           self.sensed_obj = self.sensor.get_sensed_object(0)
        #    print(self.sensed_obj)
        #    print(self.grasped)
           if not self.grasped:
               self.actuator.actuate(self.sensed_obj)
               self.actuator_1.actuate(self.sensed_obj)
               self.actuator_2.actuate(self.sensed_obj)
               self.grasped = True
               print('Grasping Sensed Object Names', self.sensed_obj)
               
            #    if str(self.sensed_obj) == 'Head' or str(self.sensed_obj) == 'Middle_6' :
            #        self.head_detect = 1
            #        self.pub_head.publish(self.head_detect)
        else:
            self.actuator.deactuate()
            self.actuator_1.deactuate()
            self.actuator_2.deactuate()
            self.grasped = False             

    



    def execute_pose_trajectory(self, traj_gen, execution_time, control_rate, peg_con):
       init_time = rospy.Time.now().to_sec()
       control_rate = rospy.Rate(control_rate)
       recorded_traj = []
       pause_start_time = 0
   
       while not rospy.is_shutdown() and not peg_con.terminate_thread:
           if not peg_con.paused_event.is_set():
               if pause_start_time == 0:
                   pause_start_time = rospy.Time.now().to_sec()
               control_rate.sleep()
               continue
   
           if pause_start_time != 0:
               pause_duration = rospy.Time.now().to_sec() - pause_start_time
               init_time += pause_duration
               pause_start_time = 0
   
           cur_time = rospy.Time.now().to_sec() - init_time
           if cur_time > execution_time:
               break
   
           execu_pos = traj_gen.get_interpolated_pose(cur_time, execution_time)
           # quanternion angle
           try:
               self.set_pos(execu_pos)
            #    print("i want to go", execu_pos)
            #    pos = self.get_pos()
            #    print("I've been", pos)

           except Exception as e:
               rospy.logwarn("Failed to set position: {}".format(str(e)))
               break
   
           record_pos = self.get_pos()
           recorded_traj.append(record_pos)
   
           control_rate.sleep()


      
        

# def test():
#     client = Client()
#     client.connect()
#     d = psmL(client)
#     b = d.get_pos()
#     print("pos0 is",b)
#     time.sleep(2)
    

    
#     # d.move()


# if __name__ == '__main__':
#     client = Client()
#     client.connect()
#     rate = rospy.Rate(50)

#     traj_gen = Traj_gen()

#     file_path = f'/home/haoyuan/Desktop/Mres_2024/peg_demo/grasping.txt'
#     traj = read_xyz_2(file_path)  # Assuming read_xyz_1 returns a list of points
#     traj = np.array(traj).T  # Convert to the required shape (3, N)
    
#     # # Create the DMP instance
#     dmp = DMP(orientation=True)
    
#     # # Imitate the trajectory
#     x = dmp.imitate(traj)  # Output: Reproduced trajectory (3*N)

#     import matplotlib.pyplot as plt
#     from mpl_toolkits.mplot3d import Axes3D

#     d = psmL(client)
#     time.sleep(2)

#     head = client.get_obj_handle('Middle_6')
#     tip_left = client.get_obj_handle('toolpitchlink_l')
    
#     head_pos = head.get_pos()
#     head_rot  = head.get_rot()
#     head_pose = np.array([head_pos.x+0.05, head_pos.y -0.015, head_pos.z+0.036,head_rot.x, head_rot.y,head_rot.z,head_rot.w])
#     print(head_pose)
#     psm_pose = d.get_pos()
    

#     traj = dmp.reproduce(initial = psm_pose, goal = head_pose)

#     print(traj[:,-1])
    
#     traj_gen.set_traj(traj)
    

   


#     while not rospy.is_shutdown():
#         a = 1
#         d.exec_pose_traj(traj_gen,5,50)
#         d.set_tip_angle(-0.15)
#         time.sleep(1)

#         pose = d.get_pos()
#         des = pose
#         des[1] = pose[1] + 0.2
#         traj = dmp.reproduce(initial = pose, goal = des)
#         traj_gen.set_traj(traj)
#         d.exec_pose_traj(traj_gen,10,50)
#         time.sleep(10)

        
#         rate.sleep()