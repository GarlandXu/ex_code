

from utilities import *
from agent.GUI_v1 import GUI
import rospy
from ik.traj_gen import Traj_gen
from DMP.dmp import dmp_discrete
from lfd.DMP.cartesian_dmp import DMP
import json

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



if __name__ == '__main__':
    # head successuful
    head_pose = np.array([0.106214,-0.18772583, 0.011314895, -0.0709440, 0.76238474, -0.6432235, -0.00003134384])
    # head_pose = np.array([0.11046533,-0.181286, -0.0351402,  0.42287332, -0.608211693, 0.670471585, 0.041527597])
    head_pose_in_world = psm2_to_world(head_pose)

    # root
    root_pose = np.array([-0.0227792, -0.1517481,  0.047762, -0.0192625, -0.5248446,0.8337056, -0.170593])
    root_pose_in_world = psm1_to_world(root_pose)

    print("\nHead in world: \n", head_pose_in_world)
    print("\nRoot in world: \n", root_pose_in_world)

    scene_file = f'scene.json'
    with open(scene_file, 'r') as file:
        scene = json.load(file)
    
    scene["head"]['position'] = [head_pose_in_world[0], head_pose_in_world[1], head_pose_in_world[2]]
    scene["head"]['orientation'] = [head_pose_in_world[3], head_pose_in_world[4], head_pose_in_world[5], head_pose_in_world[6]]

    scene["middle_1"]['position'] = [head_pose_in_world[0], head_pose_in_world[1], head_pose_in_world[2]]
    scene["middle_1"]['orientation'] = [head_pose_in_world[3], head_pose_in_world[4], head_pose_in_world[5], head_pose_in_world[6]]

    scene["middle_6"]['position'] = [root_pose_in_world[0], root_pose_in_world[1], root_pose_in_world[2]]
    scene["middle_6"]['orientation'] = [root_pose_in_world[3], root_pose_in_world[4], root_pose_in_world[5], root_pose_in_world[6]]

    scene["root"]['position'] = [root_pose_in_world[0], root_pose_in_world[1], root_pose_in_world[2]]
    scene["root"]['orientation'] = [root_pose_in_world[3], root_pose_in_world[4], root_pose_in_world[5], root_pose_in_world[6]]

    with open(scene_file, 'w') as file:
      json.dump(scene, file, indent=4)

    print("JSON file has been filled!")
