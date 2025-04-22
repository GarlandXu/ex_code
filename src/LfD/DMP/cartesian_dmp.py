import numpy as np
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import copy
import sys
sys.path.append("/home/haoyuan/catkin_ws/src/appendectomy")
from lfd.DMP.quaternion_dmp import *

class DMP():
    
    def __init__(self,N_bf=100,alphaz=4.0,betaz=1.0,orientation=True):
        
        self.alphax = 1.0
        self.alphaz = alphaz
        self.betaz = betaz
        self.N_bf = N_bf 
        self.tau = 1.0 

        self.phase = 1.0 

        self.orientation = orientation 
        if orientation:
            self.dmp_ori = QuaternionDMP(self.N_bf,self.alphax,self.alphaz,self.betaz,self.tau)

    def imitate(self, pose_demo, sampling_rate=100, oversampling=False):
        
        self.T = pose_demo.shape[0] / sampling_rate
        
        if not oversampling:
            self.N = pose_demo.shape[0]
            self.dt = self.T / self.N
            self.x = pose_demo[:,:3]
            t = np.linspace(0.0,self.T,pose_demo[:,0].shape[0])

            self.x_des = np.zeros([self.N,3])
            for d in range(3):
                x_interp = interpolate.interp1d(t,pose_demo[:,d])
                for n in range(self.N):
                    self.x_des[n,d] = x_interp(n * self.dt)
            
        else:
            self.N = 10 * pose_demo.shape[0] # 10-fold oversample
            self.dt = self.T / self.N

            t = np.linspace(0.0,self.T,pose_demo[:,0].shape[0])
            self.x_des = np.zeros([self.N,3])
            for d in range(3):
                x_interp = interpolate.interp1d(t,pose_demo[:,d])
                for n in range(self.N):
                    self.x_des[n,d] = x_interp(n * self.dt)
                
        self.c = np.ones(self.N_bf) 
        c_ = np.linspace(0,self.T,self.N_bf)
        for i in range(self.N_bf):
            self.c[i] = np.exp(-self.alphax *c_[i])

        self.h = np.ones(self.N_bf) * self.N_bf**1.5 / self.c / self.alphax

        self.dx_des = np.gradient(self.x_des,axis=0)/self.dt
        self.ddx_des = np.gradient(self.dx_des,axis=0)/self.dt

        self.x0 = self.x_des[0,:]
        self.dx0 = self.dx_des[0,:] 
        self.ddx0 = self.ddx_des[0,:]
        self.xT = self.x_des[-1,:]

        self.x = copy.deepcopy(self.x0)
        self.dx = copy.deepcopy(self.dx0)
        self.ddx = copy.deepcopy(self.ddx0)

        forcing_target_pos = self.tau*self.ddx_des - self.alphaz*(self.betaz*(self.xT-self.x_des) - self.dx_des)

        self.fit_dmp(forcing_target_pos)
        
        # Imitate orientation
        if self.orientation:
            q_des = self.dmp_ori.imitate(pose_demo[:,3:], sampling_rate, oversampling)
            return self.x_des, q_des
        else:
            return self.x_des
    
    def RBF(self, phase):

        if type(phase) is np.ndarray:
            return np.exp(-self.h*(phase[:,np.newaxis]-self.c)**2)
        else:
            return np.exp(-self.h*(phase-self.c)**2)

    def forcing_function_approx(self,weights,phase,xT=1,x0=0):

        BF = self.RBF(phase)
        if type(phase) is np.ndarray:
            return np.dot(BF,weights)*phase/np.sum(BF,axis=1)
        else:
            return np.dot(BF,weights)*phase/np.sum(BF)
    
    def fit_dmp(self,forcing_target):

        phase = np.exp(-self.alphax*np.linspace(0.0,self.T,self.N))
        BF = self.RBF(phase)
        X = BF*phase[:,np.newaxis]/np.sum(BF,axis=1)[:,np.newaxis]

        self.weights_pos = np.zeros([self.N_bf,3])

        # for d in range(3):
        #     self.weights_pos[:,d] = np.dot(np.linalg.pinv(X),forcing_target[:,d])

        regcoef = 0.01
        for d in range(3):        
            self.weights_pos[:,d] = np.dot(np.dot(np.linalg.pinv(np.dot(X.T,(X)) + \
                                    regcoef * np.eye(X.shape[1])),X.T),forcing_target[:,d].T) 

    def reset(self):
        
        self.phase = 1.0
        self.x = copy.deepcopy(self.x0)
        self.dx = copy.deepcopy(self.dx0)
        self.ddx = copy.deepcopy(self.ddx0)

        if self.orientation:
            self.dmp_ori.reset()

    def step(self, disturbance=None):
        
        disturbance_pos = np.zeros(3)
        disturbance_ori = np.zeros(3)

        if disturbance is None:
            disturbance = np.zeros(6)
        else:
            disturbance_pos = disturbance[:3]
            disturbance_ori = disturbance[3:]
        
        self.phase += (-self.alphax * self.tau * self.phase) * (self.T/self.N)
        forcing_term_pos = self.forcing_function_approx(self.weights_pos,self.phase)

        self.ddx = self.alphaz * (self.betaz * (self.xT - self.x) - self.dx) + forcing_term_pos + disturbance_pos
        self.dx += self.ddx * self.dt * self.tau
        self.x += self.dx * self.dt * self.tau

        if self.orientation:
            q, dq, ddq = self.dmp_ori.step(disturbance=disturbance_ori)
            return copy.deepcopy(self.x), copy.deepcopy(self.dx), copy.deepcopy(self.ddx), q, dq, ddq
        else:
            return copy.deepcopy(self.x), copy.deepcopy(self.dx), copy.deepcopy(self.ddx)

    def rollout(self,tau=1.0,xT=None):

        x_rollout = np.zeros([self.N,3])
        dx_rollout = np.zeros([self.N,3])
        ddx_rollout = np.zeros([self.N,3])
        x_rollout[0,:] = self.x0
        dx_rollout[0,:] = self.dx0
        ddx_rollout[0,:] = self.ddx0
        
        if xT is None:
            xT = self.xT
        
        phase = np.exp(-self.alphax*tau*np.linspace(0.0,self.T,self.N))

        # Position forcing term
        forcing_term_pos = np.zeros([self.N,3])
        for d in range(3):
            forcing_term_pos[:,d] = self.forcing_function_approx(
                self.weights_pos[:,d],phase,xT[d],self.x0[d])

        for d in range(3):
            for n in range(1,self.N):
                ddx_rollout[n,d] = self.alphaz*(self.betaz*(xT[d]-x_rollout[n-1,d]) - \
                                               dx_rollout[n-1,d]) + forcing_term_pos[n,d]
                dx_rollout[n,d] = dx_rollout[n-1,d] + tau*ddx_rollout[n-1,d]*self.dt
                x_rollout[n,d] = x_rollout[n-1,d] + tau*dx_rollout[n-1,d]*self.dt
        
        # Get orientation rollout
        if self.orientation:
            q_rollout,dq_log_rollout,ddq_log_rollout = self.dmp_ori.rollout(tau=tau)
            return x_rollout,dx_rollout,ddx_rollout, q_rollout,dq_log_rollout,ddq_log_rollout
        else:
            return x_rollout,dx_rollout,ddx_rollout
        
    def reproduce(self, initial=None, goal=None):
      if initial is not None:
          if len(initial) == 3:
              self.x0 = initial
          elif len(initial) == 7 and self.orientation:
              self.x0 = initial[:3]
              self.dmp_ori.q0 = initial[3:]
          else:
              raise ValueError("Initial must be of size 3 (position) or 7 (position + orientation).")
  
      if goal is not None:
          if len(goal) == 3:
              self.xT = goal
          elif len(goal) == 7 and self.orientation:
              self.xT = goal[:3]
              self.dmp_ori.qT = goal[3:]
          else:
              raise ValueError("Goal must be of size 3 (position) or 7 (position + orientation).")
  
      self.reset()
      self.step()
  
      
      if self.orientation:
          
          position_trajectory, _, _, orientation_trajectory, _, _ = self.rollout()
          trajectory = np.hstack((position_trajectory, orientation_trajectory)).T  # 7*N
      else:
          
          position_trajectory, _, _ = self.rollout()
          trajectory = position_trajectory.T  # 3*N
  
      return trajectory


import pandas as pd
def read_xyz_1(file_path):
    data = pd.read_csv(file_path)
    psm1_px = data['psm2_px'].astype(float)
    psm1_py = data['psm2_py'].astype(float)
    psm1_pz = data['psm2_pz'].astype(float)


    traj = np.array([psm1_px, psm1_py, psm1_pz])
    return traj

def read_xyz_2(file_path):
    data = pd.read_csv(file_path)
    psm1_px = data['psm2_px'].astype(float)
    psm1_py = data['psm2_py'].astype(float)
    psm1_pz = data['psm2_pz'].astype(float)

    psm1_ox = data['psm2_ox'].astype(float)
    psm1_oy = data['psm2_oy'].astype(float)
    psm1_oz = data['psm2_oz'].astype(float)
    psm1_ow = data['psm2_ow'].astype(float)


    traj = np.array([psm1_px, psm1_py, psm1_pz, psm1_ox, psm1_oy, psm1_oz,psm1_ow])
    return traj

def read_xyz_3(file_path):
    data = pd.read_csv(file_path)
    num_rows = len(data)
    
    # Initialize an empty array with the right shape (n, 7)
    traj = np.empty((num_rows, 7), dtype=float)
    
    # Assign position data directly
    traj[:, 0] = data['psm2_px'].astype(float)
    traj[:, 1] = data['psm2_py'].astype(float)
    traj[:, 2] = data['psm2_pz'].astype(float)
    
    # Define a fixed quaternion
    default_quaternion = np.array([0.9914, 0, 0, 0.1305])
    
    # Assign the fixed quaternion to each row
    traj[:, 3:7] = default_quaternion
    
    return traj




# Test
if __name__ == "__main__":

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    
    file_path = f'/home/haoyuan/catkin_ws/src/appendectomy/motion/approach.txt'
    traj = read_xyz_2(file_path)  
    traj = np.array(traj).T  
    
    
    print(f"Shape of traj: {np.shape(traj)}")
    
    # Create the DMP instance
    dmp = DMP(orientation=True)

    # Imitate the trajectory
    x = dmp.imitate(traj)  # Output: Reproduced trajectory (3*N)
    

    start_pos = np.array([0.78791896,0.86153268,-1,0.0985,0.1971,0.6897,0.6897])
    goal_pos = np.array([1.25   ,-0.75,   -1.5,    0.2188,0.3274,0.6500,0.6500])
    
    # Generate the reproduced trajectory
    x = dmp.reproduce(initial=start_pos, goal=goal_pos)
    print(f"Shape of reproduced trajectory: {np.shape(x)}")

    print(x[:,-1])
    
    
    # Extract positions and orientations
    positions_reproduced = x[:3, :]  # 3*N for reproduced positions
    orientations_reproduced = x[3:, :]  # 4*N for reproduced orientations (quaternions)
    
    # Transpose `traj` if necessary to ensure shape compatibility
    traj = traj.T  # Assuming traj is N*7, transpose to 7*N
    positions_original = traj[:3, :]  # 3*N for original positions
    orientations_original = traj[3:, :]  # 4*N for original orientations (quaternions)
    
    # Plot the positions in 3D space
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Original trajectory
    ax.plot(positions_original[0, :], positions_original[1, :], positions_original[2, :],
            label='Original Trajectory', linestyle='-')
    
    # Reproduced trajectory
    ax.plot(positions_reproduced[0, :], positions_reproduced[1, :], positions_reproduced[2, :],
            label='Reproduced Trajectory', linestyle='--')
    
    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Position Trajectory Comparison')
    ax.legend()
    plt.show()
    
    # Plot the quaternion components (orientation)
    fig, axs = plt.subplots(4, 1, figsize=(10, 8))
    quaternion_labels = ['x', 'y', 'z', 'w']
    for i in range(4):
        axs[i].plot(orientations_original[i, :], label=f'Original Quaternion {quaternion_labels[i]}', linestyle='-')
        axs[i].plot(orientations_reproduced[i, :], label=f'Reproduced Quaternion {quaternion_labels[i]}', linestyle='--')
        axs[i].set_title(f'Quaternion Component {quaternion_labels[i]} Over Time')
        axs[i].legend()
    plt.tight_layout()
    plt.show()