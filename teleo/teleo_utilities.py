from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import PoseStamped, Twist
import math
import os
import time
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
import numpy as np


import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D



def plot_recorded_data(file_path):
    with open(file_path, 'r') as f:
        header = f.readline().strip().split(',')  # Read header line and split by commas
    
    # Load data using pandas with proper delimiter
    data = pd.read_csv(file_path, sep=',', names=header, skiprows=1, engine='python')

    # Ensure data columns are numeric (convert to float)
    data = data.apply(pd.to_numeric, errors='coerce')

    # Extract positions (x, y, z) and quaternions (qx, qy, qz, qw) for both PSMs
    psm1_x, psm1_y, psm1_z = data['psm1_px'].values, data['psm1_py'].values, data['psm1_pz'].values
    psm2_x, psm2_y, psm2_z = 0,0,0

    psm1_qx, psm1_qy, psm1_qz, psm1_qw = data['psm1_ox'].values, data['psm1_oy'].values, data['psm1_oz'].values, data['psm1_ow'].values
    psm2_qx, psm2_qy, psm2_qz, psm2_qw = 0,0,0,0

    time = data['time'].values


    # --- Figure 1: 3D Position Trajectories ---
    fig1 = plt.figure()
    ax = fig1.add_subplot(111, projection='3d')
    ax.plot(psm1_x, psm1_y, psm1_z, label='PSM1 Position', linewidth=2)
    # ax.plot(psm2_x, psm2_y, psm2_z, label='PSM2 Position', linewidth=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Position Trajectory for PSM1 and PSM2')
    ax.legend()

    # --- Figure 2: Quaternion (Orientation) Trajectories ---
    # fig2, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    # fig2.suptitle('Quaternion Components for PSM1 and PSM2')

    # axs[0].plot(time, psm1_qx, label='PSM1 qx', linewidth=2)
    # axs[0].plot(time, psm2_qx, label='PSM2 qx', linewidth=2)
    # axs[0].set_ylabel('qx')
    # axs[0].legend()

    # axs[1].plot(time, psm1_qy, label='PSM1 qy', linewidth=2)
    # axs[1].plot(time, psm2_qy, label='PSM2 qy', linewidth=2)
    # axs[1].set_ylabel('qy')
    # axs[1].legend()

    # axs[2].plot(time, psm1_qz, label='PSM1 qz', linewidth=2)
    # axs[2].plot(time, psm2_qz, label='PSM2 qz', linewidth=2)
    # axs[2].set_ylabel('qz')
    # axs[2].legend()

    # axs[3].plot(time, psm1_qw, label='PSM1 qw', linewidth=2)
    # axs[3].plot(time, psm2_qw, label='PSM2 qw', linewidth=2)
    # axs[3].set_ylabel('qw')
    # axs[3].set_xlabel('Time')
    # axs[3].legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust for title

    # Show both figures
    plt.show()





def ambf_pos_to_arr(pos):
     position = np.array([pos.x,pos.y,pos.z])
     return position

def kdl_frame_to_pose_msg(kdl_pose):
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps

def roundRPY(RPY):
    f = 180.0 / 3.1404
    r = round(RPY[0] * f, 2)
    p = round(RPY[1] * f, 2)
    y = round(RPY[2] * f, 2)
    return r, p, y

def pose_msg_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f

def euler_from_quaternion(quan):
        w = quan[3]
        x = quan[0]
        y = quan[1]
        z = quan[2]
        s = 180 / math.pi

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1) * s
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2) * s
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4) * s

        return roll_x, pitch_y, yaw_z # in radians


def get_next_demo_file(file_path):
        i = 1
        file_name = "demo_"
        while os.path.exists(f"{file_path}/{file_name}{i}.txt"):
            i += 1
        return (f"{file_path}/{file_name}{i}.txt")

def record_data(file_path, left_gripper, right_gripper):
        left_pos = left_gripper.get_pos()
        right_pos = right_gripper.get_pos()
        left_state = 1
        right_state = 1

        record_data = (f"{time.time()}, {left_pos[3]}, {left_pos[4]}, {left_pos[5]}, {left_pos[6]}, "
                       f"{left_pos[0]}, {left_pos[1]}, {left_pos[2]}, "
                       f"{right_pos[3]}, {right_pos[4]}, {right_pos[5]}, {right_pos[6]}, "
                       f"{right_pos[0]}, {right_pos[1]}, {right_pos[2]}, "
                       f"{left_state}, {right_state}\n")

        header = ("time,psm1_ox,psm1_oy,psm1_oz,psm1_ow,"
                  "psm1_px,psm1_py,psm1_pz,"
                  "psm2_ox,psm2_oy,psm2_oz,psm2_ow,"
                  "psm2_px,psm2_py,psm2_pz,"
                  "psm1_state,psm2_state\n")

        file_is_empty = not os.path.isfile(file_path) or os.path.getsize(file_path) == 0

        with open(file_path, 'a') as file:
            if file_is_empty:
                file.write(header)
            file.write(record_data)


def record_left_data(file_path, left_gripper):
        left_pos = left_gripper.get_pos()
        left_state = 1
        right_state = 1

        record_data = (f"{time.time()}, {left_pos[3]}, {left_pos[4]}, {left_pos[5]}, {left_pos[6]}, "
                       f"{left_pos[0]}, {left_pos[1]}, {left_pos[2]}, "
                       f"{left_state}, {right_state}\n")

        header = ("time,psm1_ox,psm1_oy,psm1_oz,psm1_ow,"
                  "psm1_px,psm1_py,psm1_pz,"
                  "psm2_ox,psm2_oy,psm2_oz,psm2_ow,"
                  "psm1_state,psm2_state\n")

        file_is_empty = not os.path.isfile(file_path) or os.path.getsize(file_path) == 0

        with open(file_path, 'a') as file:
            if file_is_empty:
                file.write(header)
            file.write(record_data)
