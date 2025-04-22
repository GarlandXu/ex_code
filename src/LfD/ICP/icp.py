import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ICP:
    def __init__(self, tau=10e-6):
        self.tau = tau

    def fit(self, source_pts, target_pts):
        k = 0
        current_pts = source_pts.copy()
        last_rmse = 0
        t = np.zeros((3, 1))
        R = np.eye(3, 3)

        while True:
            neigh_pts = self._find_neighbor_points(current_pts, target_pts)
            R, t = self._register_points(source_pts, neigh_pts)
            current_pts = self._apply_transformation(source_pts, R, t)
            rmse = self._compute_rmse(current_pts, neigh_pts)

            if np.abs(rmse - last_rmse) < self.tau:
                break
            last_rmse = rmse
            k += 1

        return R, t, k

    def _compute_rmse(p1, p2):
        return np.sum(np.sqrt(np.sum((p1 - p2) ** 2, axis=0)))

    def _apply_transformation(pts, R, t):
        return np.dot(R, pts) + t

    def _register_points(p1, p2):
        u1 = np.mean(p1, axis=1).reshape((3, 1))
        u2 = np.mean(p2, axis=1).reshape((3, 1))
        pp1 = p1 - u1
        pp2 = p2 - u2
        W = np.dot(pp1, pp2.T)
        U, _, Vh = np.linalg.svd(W)
        R = np.dot(U, Vh).T
        if np.linalg.det(R) < 0:
            Vh[2, :] *= -1
            R = np.dot(U, Vh).T
        t = u2 - np.dot(R, u1)
        return R, t

    def _find_neighbor_points(source, target):
        from sklearn.neighbors import KDTree
        n = source.shape[1]
        kdt = KDTree(target.T, leaf_size=30, metric='euclidean')
        index = kdt.query(source.T, k=1, return_distance=False).reshape((n,))
        return target[:, index]

def load_trajectory(filepath):
    with open(filepath, 'r') as file:
        headers = file.readline().strip().split(',')
        data = np.loadtxt(file, delimiter=',')
    
    # Extract required columns
    time_idx = headers.index('time')
    px_idx = headers.index('psm2_px')
    py_idx = headers.index('psm2_py')
    pz_idx = headers.index('psm2_pz')

    time = data[:, time_idx].reshape(1, -1)
    points = data[:, [px_idx, py_idx, pz_idx]].T
    return time, points

def save_trajectory(filepath, time, points):
    data = np.hstack((time.T, points.T))
    header = "time,psm2_px,psm2_py,psm2_pz"
    np.savetxt(filepath, data, delimiter=',', header=header, comments='')

def plot_trajectories(trajectories, title, colors=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i, (name, traj) in enumerate(trajectories.items()):
        c = colors[i % len(colors)] if colors else None
        ax.plot(traj[0], traj[1], traj[2], label=name, color=c)
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def register_trajectories(input_folder, output_folder, reference_file):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    icp = ICP(tau=1e-10)

    reference_path = os.path.join(input_folder, reference_file)
    ref_time, ref_points = load_trajectory(reference_path)

    original_trajectories = {reference_file: ref_points}
    transformed_trajectories = {}

    for filename in os.listdir(input_folder):
        if filename == reference_file or not filename.endswith('.txt'):
            continue

        file_path = os.path.join(input_folder, filename)
        time, points = load_trajectory(file_path)

        # Store original
        original_trajectories[filename] = points

        # Apply ICP
        R, t, _ = icp.fit(points, ref_points)
        transformed_points = icp._apply_transformation(points, R, t)
        transformed_trajectories[filename] = transformed_points

        # Save transformed trajectory
        output_path = os.path.join(output_folder, filename)
        save_trajectory(output_path, time, transformed_points)
        print(f"Registered and saved: {filename}")

    # Plot all original trajectories
    plot_trajectories(original_trajectories, "Original Trajectories")

    # Plot all transformed trajectories
    transformed_trajectories[reference_file] = ref_points  # Include reference in transformed for visualization
    plot_trajectories(transformed_trajectories, "Transformed Trajectories")



# main fcn
input_folder = "/Users/garlandxu/Desktop/lfd/test"  
output_folder = "/Users/garlandxu/Desktop/lfd/output" 
reference_file = "/Users/garlandxu/Desktop/lfd/data/demo_2.txt"  


register_trajectories(input_folder, output_folder, reference_file)
