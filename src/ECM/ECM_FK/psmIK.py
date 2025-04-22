from PyKDL import Vector, Rotation, Frame, dot
Your_path
import numpy as np
import math
from psmFK import *
import rospy


# THIS IS THE IK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. CHECK THE FK
# FILE TO FIND THE CORRECT DH PARAMS BASED ON THE FRAME ATTACHMENT IN THE DVRK MANUAL

T_PalmJoint_0 = None


def get_T_PalmJoint_0():
    global T_PalmJoint_0
    return T_PalmJoint_0


def compute_IK(T_7_0):
    global T_PalmJoint_0

    # hidden paras
    L_pitch2yaw = x
    L_yaw2ctrlpnt = x
    L_tool2rcm_offset = x

    # Pinch Joint
    T_PinchJoint_7 = Frame(Rotation.RPY(0, 0, 0), L_yaw2ctrlpnt * Vector(0.0, 0.0, -1.0))
    # Pinch Joint in Origin
    T_PinchJoint_0 = T_7_0 * T_PinchJoint_7
   
    R_0_PinchJoint = T_PinchJoint_0.M.Inverse()
    P_PinchJoint_local = R_0_PinchJoint * T_PinchJoint_0.p
   
    N_PalmJoint_PinchJoint = -P_PinchJoint_local
    N_PalmJoint_PinchJoint[0] = 0
    N_PalmJoint_PinchJoint.Normalize()

    
    # print("N_PalmJoint_PinchJoint: ", round_vec(N_PalmJoint_PinchJoint))
    T_PalmJoint_PinchJoint = Frame(Rotation.RPY(0, 0, 0), N_PalmJoint_PinchJoint * L_pitch2yaw)
    # print("P_PalmJoint_PinchJoint: ", round_vec(T_PalmJoint_PinchJoint.p))
  
    T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

    # print("P_PalmJoint_0: ", round_vec(T_PalmJoint_0.p))
    # print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
    # print("Point Along the SHAFT: ", T_PalmJoint_0.p)

    # Calculate insertion_depth to check if the tool is past the RCM
    insertion_depth = T_PalmJoint_0.p.Norm()
   
    xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print ('XZ Diagonal: ', xz_diagonal)

    yz_diagonal = math.sqrt(T_PalmJoint_0.p[1] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print('YZ Diagonal: ', yz_diagonal)

    j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

    # j2 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / yz_diagonal)
    j2 = -math.atan2(T_PalmJoint_0.p[1], xz_diagonal)

    j3 = insertion_depth + L_tool2rcm_offset

    # Calculate j4
    cross_palmlink_x7_0 = T_7_0.M.UnitX() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

    # To get j4, compare the above vector with Y axes of T_3_0
    T_3_0 = convert_mat_to_frame(compute_FK([j1, j2, j3]))
    j4 = get_angle(cross_palmlink_x7_0, T_3_0.M.UnitY(), up_vector=-T_3_0.M.UnitZ())

    # Calculate j5
    # This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
    T_4_0 = convert_mat_to_frame(compute_FK([j1, j2, j3, j4]))
    j5 = get_angle(T_PinchJoint_0.p - T_PalmJoint_0.p, T_4_0.M.UnitZ(), up_vector=-T_4_0.M.UnitY())

    # Calculate j6
    # This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
    T_5_0 = convert_mat_to_frame(compute_FK([j1, j2, j3, j4, j5]))
    j6 = get_angle(T_7_0.M.UnitZ(), T_5_0.M.UnitX(), up_vector=-T_5_0.M.UnitY())

    # str = '\n**********************************'*3
    # print(str)
    # print("Joint 1: ", round(j1, 3))
    # print("Joint 2: ", round(j2, 3))
    # print("Joint 3: ", round(j3, 3))
    # print("Joint 4: ", round(j4, 3))
    # print("Joint 5: ", round(j5, 3))
    # print("Joint 6: ", round(j6, 3))

    # T_7_0_req = convert_frame_to_mat(T_7_0)
    # T_7_0_req = round_transform(T_7_0_req, 3)
    # print('Requested Pose: \n', T_7_0_req)
    # T_7_0_computed = compute_FK([j1, j2, j3, j4, j5, j6, 0])
    # round_transform(T_7_0_computed, 3)
    # print('Computed Pose: \n', T_7_0_computed)

    return [j1, j2, j3, j4, j5, j6]


