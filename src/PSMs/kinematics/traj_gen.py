from utilities import *

class Traj_gen():
    def __init__(self):
        self.demo = 0
        self.traj = None
        self.z_limit = 0.4


    def read_from_file(self, file_path):
        traj = read_xyz_1(file_path)
        self.traj = np.array(traj)

    def get_traj(self):
        return self.traj
    
    def set_traj(self,traj):
        traj = self.viture_fixture(traj)
        self.traj = traj
    
    def set_pos_traj(self,traj):
        traj = self.viture_fixture(traj)
        traj[3, :] = traj[3, 0]  
        traj[4, :] = traj[4, 0]  
        traj[5, :] = traj[5, 0]  
        traj[6, :] = traj[6, 0]  

        self.traj = traj

    def viture_fixture(self, traj):
        traj = np.array(traj)
        # traj[2,:] = np.maximum(traj[2,:],-1.45)
        return traj
    
    def get_interpolated_pos(self, cur_time, execution_time):
        # time points
        times = np.linspace(0, execution_time, num=self.traj.shape[1])
        
        if cur_time < times[0]:
            pos = self.traj[:, 0]
        elif cur_time > times[-1]:
            pos = self.traj[:, -1]
        else:
            x_interp = np.interp(cur_time, times, self.traj[0, :])
            y_interp = np.interp(cur_time, times, self.traj[1, :])
            z_interp = np.interp(cur_time, times, self.traj[2, :])
            pos = [x_interp, y_interp, z_interp]
        return pos
    
    
    def get_interpolated_pose(self, cur_time, execution_time):
        # time points
        times = np.linspace(0, execution_time, num=self.traj.shape[1])
        
        if cur_time < times[0]:
            pos = self.traj[:, 0]
        elif cur_time > times[-1]:
            pos = self.traj[:, -1]
        else:
            x_interp = np.interp(cur_time, times, self.traj[0, :])
            y_interp = np.interp(cur_time, times, self.traj[1, :])
            z_interp = np.interp(cur_time, times, self.traj[2, :])

            qx_interp = np.interp(cur_time, times, self.traj[3, :])
            qy_interp = np.interp(cur_time, times, self.traj[4, :])
            qz_interp = np.interp(cur_time, times, self.traj[5, :])
            qw_interp = np.interp(cur_time, times, self.traj[6, :])
            pos = [x_interp, y_interp, z_interp, qx_interp, qy_interp, qz_interp, qw_interp]
        return pos
    

