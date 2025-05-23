import numpy as np
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import copy

class QuaternionDMP():
    def __init__(self,N_bf=20,alphax=1.0,alphaz=12,betaz=3,tau=1.0):

        self.alphax = alphax
        self.alphaz = alphaz
        self.betaz = betaz
        self.N_bf = N_bf 
        self.tau = tau

        self.phase = 1.0 

    def imitate(self,demo_trajectory, sampling_rate=100, oversampling=True):
        
        self.T = demo_trajectory.shape[0] / sampling_rate
        
        if not oversampling:
            self.N = demo_trajectory.shape[0]
            self.dt = self.T / self.N
            self.q_des = demo_trajectory
            
        else:
            self.N = 10 * demo_trajectory.shape[0] # 10-fold oversample
            self.dt = self.T / self.N
            t = np.linspace(0.0,self.T,demo_trajectory[:,0].shape[0])
            self.q_des = np.zeros([self.N,4])
            slerp = Slerp(t,R.from_quat(demo_trajectory[:]))
            self.q_des = slerp(np.linspace(0.0,self.T,self.N)).as_quat()
        
        # Centers of basis functions 
        self.c = np.ones(self.N_bf) 
        c_ = np.linspace(0,self.T,self.N_bf)
        for i in range(self.N_bf):
            self.c[i] = np.exp(-self.alphax *c_[i])

        self.h = np.ones(self.N_bf) * self.N_bf**1.5 / self.c / self.alphax

        self.dq_des_log = self.quaternion_diff(self.q_des)
        self.ddq_des_log = np.zeros(self.dq_des_log.shape)
        for d in range(3):
            self.ddq_des_log[:,d] = np.gradient(self.dq_des_log[:,d])/self.dt

        # Initial and final orientation
        self.q0 = self.q_des[0,:]
        self.dq0_log = self.dq_des_log[0,:] 
        self.ddq0_log = self.ddq_des_log[0,:]
        self.qT = self.q_des[-1,:]

        # Initialize the DMP
        self.q = copy.deepcopy(self.q0)
        self.dq_log = copy.deepcopy(self.dq0_log)
        self.ddq_log = copy.deepcopy(self.ddq0_log)

        # Evaluate the forcing term
        forcing_target = np.zeros([self.N,3]) 
        for n in range(self.N):
            forcing_target[n,:] = self.tau*self.ddq_des_log[n,:] - \
                                    self.alphaz*(self.betaz*self.logarithmic_map(
                                    self.quaternion_error(self.qT,self.q_des[n,:])) - self.dq_des_log[n,:])

        self.fit_dmp(forcing_target)
        
        return self.q_des

    def quaternion_conjugate(self,q):
        return q * np.array([1.0,-1.0,-1.0,-1.0])

    def quaternion_product(self,q1,q2):
        q1 = np.array(q1)
        q2 = np.array(q2)

        q12 = np.zeros(4)
        q12[0] = q1[0]*q2[0] - np.dot(q1[1:],q2[1:])
        q12[1:] = q1[0]*q2[1:] + q2[0]*q1[1:] + np.cross(q1[1:],q2[1:])
        return q12

    def quaternion_error(self,q1,q2):
        return self.quaternion_product(q1,self.quaternion_conjugate(q2))

    def exponential_map(self,r):

        theta = np.linalg.norm(r)
        if theta == 0.0:
            return np.array([1.0, 0.0, 0.0, 0.0])

        n = r / np.linalg.norm(r)

        q = np.zeros(4)
        q[0] = np.cos(theta / 2.0)
        q[1:] = np.sin(theta/ 2.0) * n

        return q

    def logarithmic_map(self,q):

        if np.linalg.norm(q[1:]) < np.finfo(float).eps:
            return np.zeros(3)

        n = q[1:] / np.linalg.norm(q[1:])
        theta = 2.0 * np.arctan2(np.linalg.norm(q[1:]),q[0])

        return theta*n

    def quaternion_diff(self,q):

        dq_log = np.zeros([q.shape[0], 3])
        dq_log[0,:] = self.logarithmic_map(self.quaternion_error(q[1,:], q[0,:])) / self.dt
        for n in range(1, q.shape[0]-1):
            dq_log[n,:] = self.logarithmic_map(self.quaternion_error(q[n+1,:], q[n-1,:])) / (2.0*self.dt)
        dq_log[-1,:] = self.logarithmic_map(self.quaternion_error(q[-1,:], q[-2,:])) / self.dt

        return dq_log

    def RBF(self, phase):

        if type(phase) is np.ndarray:
            return np.exp(-self.h*(phase[:,np.newaxis]-self.c)**2)
        else:
            return np.exp(-self.h*(phase-self.c)**2)

    def forcing_function_approx(self,weights,phase):

        BF = self.RBF(phase)
        if type(phase) is np.ndarray:
            return np.dot(BF,weights)*phase/np.sum(BF,axis=1)
        else:
            return np.dot(BF,weights)*phase/np.sum(BF)

    def fit_dmp(self,forcing_target):

        phase = np.exp(-self.alphax*np.linspace(0.0,self.T,self.N))
        BF = self.RBF(phase)
        X = BF*phase[:,np.newaxis]/np.sum(BF,axis=1)[:,np.newaxis]
        dof = forcing_target.shape[1]

        self.weights = np.zeros([self.N_bf,dof])
        for d in range(dof):
            self.weights[:,d] = np.dot(np.linalg.pinv(X),forcing_target[:,d])

    def reset(self):
        
        self.phase = 1.0
        self.q = copy.deepcopy(self.q0)
        self.dq_log = copy.deepcopy(self.dq0_log)
        self.ddq_log = copy.deepcopy(self.ddq0_log)

    def step(self, disturbance=None):
        
        if disturbance is None:
            disturbance = np.zeros(3)
        
        self.phase += (-self.alphax * self.tau * self.phase) * (self.T/self.N)
        forcing_term = self.forcing_function_approx(self.weights,self.phase)
        
        self.ddq_log = self.alphaz*(self.betaz*self.logarithmic_map(
                self.quaternion_error(self.qT,self.q)) - self.dq_log) + forcing_term + disturbance
        self.dq_log += self.ddq_log * self.dt * self.tau
        self.q = self.quaternion_product(self.exponential_map(self.tau*self.dq_log*self.dt),self.q)
        
        return copy.deepcopy(self.q), copy.deepcopy(self.dq_log), copy.deepcopy(self.ddq_log)

    def rollout(self,tau=1.0):

        q_rollout = np.zeros([self.N,4])
        dq_log_rollout = np.zeros([self.N,3])
        ddq_log_rollout = np.zeros([self.N,3])
        q_rollout[0,:] = self.q0
        dq_log_rollout[0,:] = self.dq0_log
        ddq_log_rollout[0,:] = self.ddq0_log
        
        phase = np.exp(-self.alphax*tau*np.linspace(0.0,self.T,self.N))

        forcing_term = np.zeros([self.N,3])
        for d in range(3):
            forcing_term[:,d] = self.forcing_function_approx(self.weights[:,d],phase)

        for n in range(1,self.N):            
            ddq_log_rollout[n,:] = self.alphaz*(self.betaz*self.logarithmic_map(
                self.quaternion_error(self.qT,q_rollout[n-1,:])) - dq_log_rollout[n-1,:]) + \
                forcing_term[n,:]

            dq_log_rollout[n,:] = dq_log_rollout[n-1,:] + tau*ddq_log_rollout[n-1,:]*self.dt
            q_rollout[n,:] = self.quaternion_product(self.exponential_map(tau*dq_log_rollout[n-1,:]*self.dt),q_rollout[n-1,:])

        return q_rollout,dq_log_rollout,ddq_log_rollout


# Test

# if __name__ == "__main__":
#     import numpy as np
#     import matplotlib.pyplot as plt
    
#     # Load the demonstration trajectory
#     with open('quaternion_trajectory.npy', 'rb') as f:
#         demo_trajectory = np.load(f)
    
#     # Initialize the Quaternion DMP
#     dmp = QuaternionDMP(N_bf=100)  # 20 basis functions
#     q_des = dmp.imitate(demo_trajectory)
    
#     # Define new start and goal orientations (example quaternions)
#     new_goal  = np.array([0.2188,0.3274,0.6500,0.6500])  # Identity quaternion
#     new_start = np.array([0.0985,0.1971,0.6897,0.6897])  # 90-degree rotation about X-axis
    
#     # Update the DMP with new start and goal positions
#     dmp.q0 = new_start
#     dmp.qT = new_goal
#     dmp.reset()  # Reset the DMP state to match the new start position
#     dmp.step()
    
#     # Generate the new trajectory
#     q_rollout, dq_log_rollout, ddq_log_rollout = dmp.rollout()
#     print(q_rollout[-1,:])
   
#     # Visualize the original and reproduced trajectories
#     fig = plt.figure(figsize=(18, 3))
#     for d in range(4):  # Plot each quaternion component
#         plt.subplot(1, 4, d+1)
#         plt.plot(q_des[:, d], label='Original Demonstration')
#         plt.plot(q_rollout[:, d], '--', label='Reproduced Trajectory')
#         plt.title(f'Quaternion Component {d}')
#         plt.legend()
#     plt.show()


    