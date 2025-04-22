from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
import pandas as pd
from scipy.spatial.transform import Rotation as R
import math
import pickle

PI = np.pi
PI_2 = np.pi/2


def read_xyz_1(file_path):
    data = pd.read_csv(file_path)
    psm1_px = data['psm2_px'].astype(float)
    psm1_py = data['psm2_py'].astype(float)
    psm1_pz = data['psm2_pz'].astype(float)

    traj = np.array([psm1_px, psm1_py, psm1_pz])
    return traj

# The up vector is useful to define angles > PI. Since otherwise
# this method will only report angles <= PI.
def get_angle(vec_a, vec_b, up_vector=None):
    vec_a.Normalize()
    vec_b.Normalize()
    cross_ab = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        angle = 0.0
        # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        angle = np.pi
    else:
        angle = math.acos(vdot)

    if up_vector is not None:
        same_dir = np.sign(dot(cross_ab, up_vector))
        if same_dir < 0.0:
            angle = -angle

    return angle


def round_mat(mat, rows, cols, precision=4):
    for i in range(0, rows):
        for j in range(0, cols):
            mat[i, j] = round(mat[i, j], precision)
    return mat


def round_vec(vec, precision=4):
    for i in range(3):
        vec[i] = round(vec[i], precision)
    return vec


def round_transform(mat, precision=4):
    return round_mat(mat, 4, 4, precision)


def convert_frame_to_mat(frame):
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]

    for i in range(3):
        np_mat[i, 3] = frame.p[i]

    return np_mat


def convert_mat_to_frame(mat):
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame[(i, j)] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame


def pose_from_traj(data):
    data = np.array(data)

    # Extract translation and quaternion
    x, y, z, qx, qy, qz, qw = data

    # Compute the rotation matrix from quaternion
    rotation_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

    # Construct the 4x4 pose matrix
    pose = np.eye(4)
    pose[:3, :3] = rotation_matrix
    pose[:3, 3] = [x, y, z]

    return pose



def traj_from_pose(pose):
    x,y,z = pose[0,3], pose[1,3], pose[2,3]
    rot = pose[:3,:3]
    rotation = R.from_matrix(rot)
    quat = rotation.as_quat()

    traj  = np.array([x,y,z,quat[0],quat[1],quat[2],quat[3]])

    return traj




def calculate_transform(psm1_p1, psm1_p2, psm2_p1, psm2_p2):
    psm1_v1 = np.array(psm1_p1) - np.array(psm1_p2)
    psm2_v2 = np.array(psm2_p1) - np.array(psm2_p2)

    dot_product = np.dot(psm1_v1, psm2_v2)
    cross_product = np.cross(psm1_v1, psm2_v2)

    magnitude_v1 = np.linalg.norm(psm1_v1)
    magnitude_v2 = np.linalg.norm(psm2_v2)

    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    theta = np.arccos(cos_theta)

    if np.linalg.norm(cross_product) != 0:
        rotation_axis = cross_product / np.linalg.norm(cross_product)
    else:
        rotation_axis = np.array([0, 0, 1])  

    K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                  [rotation_axis[2], 0, -rotation_axis[0]],
                  [-rotation_axis[1], rotation_axis[0], 0]])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

    T = np.array(psm2_p1) - np.dot(R, np.array(psm1_p1))

    trans_matrix = np.eye(4)
    trans_matrix[:3, :3] = R
    trans_matrix[:3, 3] = T

    return trans_matrix








def Rotx(x):
    cos = math.cos(x)
    sin = math.sin(x)    
    Rx = np.matrix([[1, 0, 0],
                   [0,cos,-sin],
                   [0,sin,cos]])
    return Rx
    
def Roty(y):
    cos = math.cos(y)
    sin = math.sin(y)    
    Ry = np.matrix([[cos, 0, sin],
                   [0,1,0],
                   [-sin,0,cos]])
    return Ry

def Rotz(z):
    cos = math.cos(z)
    sin = math.sin(z)    
    Rz = np.matrix([[cos, -sin, 0],
                   [sin,cos,0],
                   [0,0,1]])
    return Rz

def return_Ryz():
    Ry = np.matrix([[-1, 0, 0],
                    [0, 1, 0],
                    [0, 0, -1]])
    
    Rz = np.matrix([[0, 1, 0],
                    [-1,0,0],
                    [0, 0, 1]])
    
    return Ry,Rz

def rotate_z2x(Trans_matrix):
    
    T_r = np.matrix([[0,0, -1, 0],
                     [0,1,0,0],
                     [1,0,0,0],
                     [0,0,0,1]])  
    
    T_modified = Trans_matrix*T_r
    
    return  T_modified
    
def rotate_x_90(Trans_matrix):
    
    T_r = np.matrix([[1,0, 0, 0],
                    [0,0,-1,0],
                    [0,1,0,0],
                    [0,0,0,1]])  

    T_modified = Trans_matrix*T_r

    return  T_modified   


def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])  
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])                 
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R  

# set inital posture for PSM 
def set_inital_L_pos(_handle_left,inital_joint_1,inital_joint_2,inital_joint_3):
    _handle_left.set_joint_pos('baselink-yawlink_l', inital_joint_1)
    _handle_left.set_joint_pos('yawlink-pitchbacklink_l', inital_joint_2)
    _handle_left.set_joint_pos('pitchendlink-maininsertionlink_l', inital_joint_3)
    _handle_left.set_joint_pos('maininsertionlink-toolrolllink_l',0)
    _handle_left.set_joint_pos('toolrolllink-toolpitchlink_l',0) 
    _handle_left.set_joint_pos('toolpitchlink-toolgripper1link_l', -0.35)
    _handle_left.set_joint_pos('toolpitchlink-toolgripper2link_l', -0.35)   
    
def set_inital_R_pos(_handle_right,inital_joint_1,inital_joint_2,inital_joint_3):
    _handle_right.set_joint_pos('baselink-yawlink_r', inital_joint_1)
    _handle_right.set_joint_pos('yawlink-pitchbacklink_r', inital_joint_2)
    _handle_right.set_joint_pos('pitchendlink-maininsertionlink_r', inital_joint_3)
    _handle_right.set_joint_pos('maininsertionlink-toolrolllink_r',0)
    _handle_right.set_joint_pos('toolrolllink-toolpitchlink_r',0) 
    _handle_right.set_joint_pos('toolpitchlink-toolgripper1link_r', -0.35)
    _handle_right.set_joint_pos('toolpitchlink-toolgripper2link_r', -0.35)   

#get current joint value of PSM    
def get_PSML_pos(_handle_left):
        j_1 = _handle_left.get_joint_pos('baselink-yawlink_l')
        j_2 = _handle_left.get_joint_pos('yawlink-pitchbacklink_l')
        j_3 = _handle_left.get_joint_pos('pitchendlink-maininsertionlink_l')
        j_4 = _handle_left.get_joint_pos('maininsertionlink-toolrolllink_l')
        j_5 = _handle_left.get_joint_pos('toolrolllink-toolpitchlink_l')
        j_6 = _handle_left.get_joint_pos('toolpitchlink-toolgripper1link_l')
        j_7 = _handle_left.get_joint_pos('toolpitchlink-toolgripper2link_l')  
        
        joint_state = np.array([j_1,j_2,j_3,j_4,j_5,j_6,j_7], dtype=np.float32)  
        
        return joint_state
    
def get_PSMR_pos(_handle_left):
        j_1 = _handle_left.get_joint_pos('baselink-yawlink_r')
        j_2 = _handle_left.get_joint_pos('yawlink-pitchbacklink_r')
        j_3 = _handle_left.get_joint_pos('pitchendlink-maininsertionlink_r')
        j_4 = _handle_left.get_joint_pos('maininsertionlink-toolrolllink_r')
        j_5 = _handle_left.get_joint_pos('toolrolllink-toolpitchlink_r')
        j_6 = _handle_left.get_joint_pos('toolpitchlink-toolgripper1link_r')
        j_7 = _handle_left.get_joint_pos('toolpitchlink-toolgripper2link_r')  
        
        joint_state = np.array([j_1,j_2,j_3,j_4,j_5,j_6,j_7], dtype=np.float32)  
        
        return joint_state

# those are for ambf only!!!!!!!!!!!!!!!!!!!!!!!!!
# in real world, you only get measurements from the tip in their psm frame
def get_PSM_l():
    
    PSM_pos = [2.69932,0.69109,2.36776]
    PSM_rpy = [0.0, 0.0, -2.5]    
    
    t = np.array([PSM_pos[0],PSM_pos[1],PSM_pos[2]]).transpose()
    R =eulerAnglesToRotationMatrix(PSM_rpy)
      

    T_psml_g = np.empty((4, 4))
    T_psml_g[:3, :3] = R
    T_psml_g[:3,  3] = t
    T_psml_g[3, :] = [0, 0, 0, 1] 
    # print('T_psml_g',T_psml_g)   
    
    return T_psml_g   

def get_PSM_r():
    
    PSM_pos = [2.32087,-0.68371,2.22747]
    PSM_rpy = [0.0, 0.0, -0.45371999999999996]    
    
    t = np.array([PSM_pos[0],PSM_pos[1],PSM_pos[2]]).transpose()
    R =eulerAnglesToRotationMatrix(PSM_rpy)
      

    T_psmr_g = np.empty((4, 4))
    T_psmr_g[:3, :3] = R
    T_psmr_g[:3,  3] = t
    T_psmr_g[3, :] = [0, 0, 0, 1] 
    # print('T_psml_g',T_psml_g)   
    
    return T_psmr_g 

def obj_in_PSM_l(obj_world):
    pose_obj_world = pose_from_traj(obj_world)
    T_psm_l = get_PSM_l()
    pose_obj_psml = np.dot(np.linalg.inv(T_psm_l),pose_obj_world) 
    obj_psml = traj_from_pose(pose_obj_psml)

    return obj_psml

def obj_in_PSM_r(obj_world):
    pose_obj_world = pose_from_traj(obj_world)
    T_psm_r = get_PSM_r()
    pose_obj_psmr = np.dot(np.linalg.inv(T_psm_r),pose_obj_world) 
    obj_psmr = traj_from_pose(pose_obj_psmr)

    return obj_psmr

def psml_in_world(psml_point):
     pose_psml_world = pose_from_traj(psml_point)
     T_psml = get_PSM_l()
     psml_in_world = np.dot(T_psml, pose_psml_world)

     traj_psml_world = traj_from_pose(psml_in_world)

     return traj_psml_world

def psmr_in_world(psmr_point):
     pose_psmr_world = pose_from_traj(psmr_point)
     T_psmr = get_PSM_r()
     psmr_in_world = np.dot(T_psmr, pose_psmr_world)

     traj_psmr_world = traj_from_pose(psmr_in_world)

     return traj_psmr_world


def show_ECM(_handle):
    q = [0.44148001074790955, -0.0006888926145620644, 0.6217290163040161, -2.2059054374694824]
    
    j_1 = _handle.set_joint_pos('baselink-yawlink',q[0])
    j_2 = _handle.set_joint_pos('yawlink-pitchbacklink',q[1])
    j_3 = _handle.set_joint_pos('pitchendlink-maininsertionlink',q[2])
    j_4 = _handle.set_joint_pos('maininsertionlink-toollink',q[3])
    
    print("ECM has been initialized!")
    


# def traj_from_pose(pose):
#     # pose = np.array(pose)

#     # trans and rotation
#     x,y,z = pose[:3,3]
#     R = pose[:3,:3]

#     trace = np.trace(R)
#     # for numerical stability
#     if trace > 0:
#         S = np.sqrt(trace + 1.0 ) * 2
#         qw = 0.25 * S
#         qx = (R[2,1] - R[1,2]) / S
#         qy = (R[0,2] - R[2,0]) / S
#         qz = (R[1,0] - R[0,1]) / S
    
#     elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
#         S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
#         qw = (R[2,1] - R[1,2]) / S
#         qx = 0.25 * S
#         qy = (R[0,1] + R[1,0]) / S
#         qz = (R[0,2] + R[2,0]) / S


#     elif R[1,1] > R[2,2]:
#         S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
#         qw = (R[0,2] - R[2,0]) / S
#         qx = (R[0,1] + R[1,0]) / S
#         qy = 0.25 * S
#         qz = (R[1,2] + R[2,1]) / S

#     else:
#         S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
#         qw = (R[1,0] - R[0,1]) / S
#         qx = (R[0,1] + R[2,0]) / S
#         qy = (R[1,2] + R[2,1]) / S
#         qz = 0.25 * S


    
#     traj = np.array([float(x),float(y),float(z), qx,qy,qz,qw])

#     return traj



def read_pickle(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    return data
  

