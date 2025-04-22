
import math
import os
import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from PyKDL import Vector, Rotation, Frame, dot,PoseStamped



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

def quan_to_rpy(quan):
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

def rpy_to_radian(rpy):
     f = 3.1404 / 180
     r = round(rpy[0] * f, 2)
     p = round(rpy[1] * f, 2)
     y = round(rpy[2] * f, 2)
     return r, p, y

def get_next_demo_file(file_path):
        i = 1
        file_name = "demo_"
        while os.path.exists(f"{file_path}/{file_name}{i}.txt"):
            i += 1
        return (f"{file_path}/{file_name}{i}.txt")

def record_data(file_path, left_gripper, right_gripper):
        left_pos = left_gripper.get_pos()
        left_quan = left_gripper.get_quan()
        right_pos = right_gripper.get_pos()
        right_quan = right_gripper.get_quan()
        
        record_data = (f"{time.time()}, {left_quan[0]}, {left_quan[1]}, {left_quan[2]}, {left_quan[3]}, "
                       f"{left_pos[0]}, {left_pos[1]}, {left_pos[2]}, "
                       f"{right_quan[0]}, {right_quan[1]}, {right_quan[2]}, {right_quan[3]}, "
                       f"{right_pos[0]}, {right_pos[1]}, {right_pos[2]}\n")

        header = ("time,psm1_ox,psm1_oy,psm1_oz,psm1_ow,"
                  "psm1_px,psm1_py,psm1_pz,"
                  "psm2_ox,psm2_oy,psm2_oz,psm2_ow,"
                  "psm2_px,psm2_py,psm2_pz\n")

        file_is_empty = not os.path.isfile(file_path) or os.path.getsize(file_path) == 0

        with open(file_path, 'a') as file:
            if file_is_empty:
                file.write(header)
            file.write(record_data)


def dvrk_record_data(file_path, PSM1_xyz, PSM1_quan, PSM2_xyz, PSM2_quan):
        
        record_data = (f"{time.time()}, {PSM1_quan.x}, {PSM1_quan.y}, {PSM1_quan.z}, {PSM1_quan.w}, "
                       f"{PSM1_xyz.x}, {PSM1_xyz.y}, {PSM1_xyz.z}, "
                       f"{PSM2_quan.x}, {PSM1_quan.y}, {PSM1_quan.z}, {PSM1_quan.w}, "
                       f"{PSM2_xyz.x}, {PSM2_xyz.y}, {PSM2_xyz.z}\n")

        header = ("time,psm1_ox,psm1_oy,psm1_oz,psm1_ow,"
                  "psm1_px,psm1_py,psm1_pz,"
                  "psm2_ox,psm2_oy,psm2_oz,psm2_ow,"
                  "psm2_px,psm2_py,psm2_pz\n")

        file_is_empty = not os.path.isfile(file_path) or os.path.getsize(file_path) == 0

        with open(file_path, 'a') as file:
            if file_is_empty:
                file.write(header)
            file.write(record_data)

def read_xyz(file_path):
    data = pd.read_csv(file_path)
    psm1_px = data['psm1_px'].astype(float)
    psm1_py = data['psm1_py'].astype(float)
    psm1_pz = data['psm1_pz'].astype(float)

    traj = np.array([psm1_px, psm1_py, psm1_pz])
    return traj

def read_xyz_1(file_path):
    data = pd.read_csv(file_path)
    psm1_px = data['psm2_px'].astype(float)
    psm1_py = data['psm2_py'].astype(float)
    psm1_pz = data['psm2_pz'].astype(float)

    traj = np.array([psm1_px, psm1_py, psm1_pz])
    return traj

def plot_3d(traj):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj[0, :], traj[1, :], traj[2, :], label='Demo')
   
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('Trajectory of left gripper')
    plt.show()


if __name__ == "__main__":

    file_path = f'/Users/garlandxu/Desktop/lfd/data/demo_1.txt'
    traj = read_xyz(file_path)
    traj = np.array(traj)
    print(np.shape(traj))











