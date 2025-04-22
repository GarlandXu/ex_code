from ik.psmIK import *
from ambf_client import Client
import time
from joint_space_trajectory_generator import JointSpaceTrajectory

def test_ambf_psm(test_frame):


    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b
    
    # js_traj = JointSpaceTrajectory(num_joints=7, num_traj_points=1, joint_limits=joint_lims)
    # num_points = js_traj.get_num_traj_points()
    # num_joints = 6

    # for i in range(num_points):
        # test_q = js_traj.get_traj_at_point(i)
        # T_7_0 = compute_FK(test_q)
        
        # P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
        # R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
        # T_0_w = Frame(R_0_w, P_0_w)
        # T_7_w = T_0_w * convert_mat_to_frame(T_7_0)  



    computed_q = compute_IK(convert_mat_to_frame(test_frame))
    
    print(computed_q)

    b.set_joint_pos('baselink-yawlink_l', computed_q[0])
    b.set_joint_pos('yawlink-pitchbacklink_l', computed_q[1])
    b.set_joint_pos('pitchendlink-maininsertionlink_l', computed_q[2])
    b.set_joint_pos('maininsertionlink-toolrolllink_l', computed_q[3])
    b.set_joint_pos('toolrolllink-toolpitchlink_l', computed_q[4])
    b.set_joint_pos('toolpitchlink-toolgripper1link_l', computed_q[5])
    b.set_joint_pos('toolpitchlink-toolgripper2link_l', -computed_q[5])
    
    time.sleep(2)
    tip_left = c.get_obj_handle('psm_l/toolgripper1link_l')
    position_l = tip_left.get_pos()
    
    X_str = str(round(position_l.x,4))
    Y_str = str(round(position_l.y,4))
    Z_str = str(round(position_l.z,4))
    
    string = str(rospy.get_time())+"\t x= " + X_str + ",\t y= " + Y_str + ",\t z= " + Z_str
    print(string)
        

test_frame =np.matrix([[ 0.928,  0.25,   0.274,  0.012],
                       [ 0.353, -0.368, -0.86,  -0.071],
                       [-0.114,  0.896, -0.43,  -0.061],
                       [ 0, 0, 0, 1]])    

    
    
    
            

if __name__ == "__main__":
    
    c = Client('psm_ik_test')
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('psm_l/baselink_l')
   
    num_joints = 7
    joint_lims = np.zeros((num_joints, 2))
    joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
    joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
    joint_lims[2] = [0.0, 0.24]
    joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]
    joint_lims[4] = [np.deg2rad(-90), np.deg2rad(90)]
    joint_lims[5] = [np.deg2rad(-85), np.deg2rad(85)]
    joint_lims[6] = [0.0, 0.0]
    
    # test_frame_0 =np.matrix([[ 0,  1,  0,  0],
    #                          [ -1,  0,  0,  0],
    #                          [ 0,  0,  1,  0.14],
    #                          [ 0,  0, 0, 1]])
    
    # test_frame_0 =np.matrix([[-0.  ,   -0.  ,   1. ,    0. ,  ],
    #                         [-0.  ,  1.  ,  0.  ,   0.   ],
    #                         [-1.  ,  -0.   ,  -0. ,    0.014],
    #                         [ 0.  ,   0.    , 0.,     1.   ]])
    
    test_frame_0 =np.matrix(   [[   -0.4131  ,       0  ,  0.9107 ,  -0.0121],
                                   [ 0.9107  ,       0  ,  0.4131 ,  -0.0073],
                                   [ 0   , 1.0000    ,     0  , -1.4615],
                                   [ 0   ,      0    ,     0  ,  1.0000]])
    
    test_frame_1 =np.matrix([[ 0.928,  0.25,   0.274,  0.012],
                           [ 0.353, -0.368, -0.86,  -0.071],
                           [-0.114,  0.896, -0.43,  -0.061],
                           [ 0, 0, 0, 1]])        
    
    R_2_tip =np.matrix([[0 ,1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1,1]]);
    
    test_frame_0 = R_2_tip*test_frame_0
    

    for i in range(5):
        test_frame_0[2,3]=test_frame_0[2,3] - 0.5
        
        print(test_frame_0)
        
        test_ambf_psm(test_frame_0)
        
    # test_ambf_psm(test_frame_0)
    # test_ambf_psm(test_frame_1)
        
    
    input()