

import math
import numpy as np

Your_path
from identify.mrcnn.config import Config
from identify.ECM_FK.ecmFK import compute_FK


class ShapesConfig(Config):
    NAME = "shapes"

    GPU_COUNT = 1
    IMAGES_PER_GPU = 4
    NUM_CLASSES = 1 + 1  # background + 1 class

    IMAGE_MIN_DIM = 480
    IMAGE_MAX_DIM = 1024

    RPN_ANCHOR_SCALES = (8 * 6, 16 * 6, 32 * 6, 64 * 6, 128 * 6)  # anchor side in pixels

    TRAIN_ROIS_PER_IMAGE = 1  # default 32
    STEPS_PER_EPOCH = 100
    VALIDATION_STEPS = 5

class Appendix_Config(ShapesConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    

def get_app_index(mask,point_coord):
    nn = 0
    
    for i in range(480):
        for j in range(720):       
            if mask[i][j] == True:
                point_coord[0][nn] = j  
                point_coord[1][nn] = i
                nn += 1  
                # print(mask[i][j],i,j,nn)
                                                          
    return point_coord

def get_app_coord(arr,total_point,mask_coord,points):
    for i in range(total_point):
        row = int(arr[0][i])
        line = int(arr[1][i])
        
        point_list_pos = line * 720 + row
        
        mask_coord[0][i] = points[point_list_pos][0]
        mask_coord[1][i] = points[point_list_pos][1]
        mask_coord[2][i] = points[point_list_pos][2]

        x_c = sum(mask_coord[0])/total_point
        y_c = sum(mask_coord[1])/total_point
        z_c = sum(mask_coord[2])/total_point
        
        point_centroid = [x_c,y_c,z_c]

    return mask_coord, point_centroid   


def get_head_root_coord(arr,points):
    
    row = int(arr[0][0])
    line = 480 - int(arr[0][1])
    
    coord = np.empty((1,3))
    
    point_list_pos = line * 720 + row
    
    coord[0][0] = float(points[point_list_pos][0])
    coord[0][1] = float(points[point_list_pos][1])
    coord[0][2] = float(points[point_list_pos][2])

    coord_return = [coord[0][0],coord[0][1],coord[0][2]]

    return coord_return


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

def get_ECM():
    ECM_pos = [2.13393,0.03789,2.47927]
    ECM_rpy= [0.0, 0.0, -1.5708]
    joint_ECM = [0.44148001074790955, -0.0006888926145620644, 0.6217290163040161, -2.2059054374694824]
    
    ECM_EF = compute_FK(joint_ECM)

    Y_90 = np.array([[0,0,1,0],
                     [0,1,0,0],
                     [-1,0,0,0],
                     [0,0,0,1]])

    T_Cam_l = ECM_EF * Y_90
    
    t = np.array([ECM_pos[0],ECM_pos[1],ECM_pos[2]]).transpose()
    R =eulerAnglesToRotationMatrix(ECM_rpy)
    
    T_ECM_g = np.empty((4, 4))
    T_ECM_g[:3, :3] = R
    T_ECM_g[:3,  3] = t
    T_ECM_g[3, :] = [0, 0, 0, 1] 
    
    # print('T_Cam_l',T_Cam_l)   
    # print('T_ECM_g',T_ECM_g)   

    return T_Cam_l,T_ECM_g

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


def get_app_head_psml(T_Cam_l,T_ECM_g,T_psml_g,app_head_cam):
    T_Cam_l = np.squeeze(np.asarray(T_Cam_l))
    print('T_Cam_l: ',T_Cam_l) 
    
    app_head_ecm = np.dot(T_Cam_l,app_head_cam)
    print('app_head_ecm: ',app_head_ecm) 
    
    T_ecm2psm = np.dot(np.linalg.inv(T_ECM_g),T_psml_g)
    print('T_ecm2psm: ',T_ecm2psm)    

    app_head_psml = np.dot(np.linalg.inv(T_ecm2psm),app_head_ecm) 
    
    return app_head_psml  

