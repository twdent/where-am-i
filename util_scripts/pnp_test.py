

import numpy as np
import cv2
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])



# Define 3D world points
match_3d_pts = np.array([
    [-3.52757815, 1.11143963, -0.07943038],
    [-3.58154688, 1.30744047, -0.07588769],
    [-3.63550402, 1.41612563, -0.3298474],
    [-3.68867988, 1.6108313, -0.32407472],
    [-3.55463081, 1.2369827, 0.26009885],
    [-3.60819871, 1.43110107, 0.2622637],
    [-3.68241879, 1.6964233, -0.07329942],
    [-3.66203988, 1.62505065, 0.26450664]
], dtype=np.float32)

# Define corresponding 2D image points
mkpts0 = np.array([
    [1000., 400.],
    [775., 398.],
    [739., 456.],
    [677., 455.],
    [801., 322.],
    [740., 322.],
    [651., 397.],
    [678., 321.]
], dtype=np.float32)


# Load camera intrinsic parameters (You need to provide your own)
camera_matrix = np.array([377.1323930964233, 0.0, 720, 
                            0.0, 376.76480664462014, 540, 
                            0.0, 0.0, 1.0]).reshape(3,3)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Estimate camera pose using solvePnPRansac
success, rvec, tvec, inliers = cv2.solvePnPRansac(match_3d_pts, mkpts0, camera_matrix, dist_coeffs)

R, _ = cv2.Rodrigues(rvec)

# Convert to world coordinate system
C_t_cw = tvec
C_t_wc = -C_t_cw
R_cw = R
R_wc = R_cw.T
#camera translation in world frame
w_t_wc = np.matmul(R_wc, C_t_wc)

#plot in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(match_3d_pts[:,0], match_3d_pts[:,1], match_3d_pts[:,2], c='b', marker='o')

#plot camera pos in world frame
ax.scatter(w_t_wc[0], w_t_wc[1], w_t_wc[2], c='r', marker='o')

#plot camera z axis in world frame
c_camera_z_axis = np.array([0, 0, 1])
w_camera_z_axis = np.matmul(R_wc, c_camera_z_axis)
ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_z_axis[0], w_camera_z_axis[1], w_camera_z_axis[2], length=1, normalize=True, color='r')
#plot camera x axis in world frame
c_camera_x_axis = np.array([1, 0, 0])
w_camera_x_axis = np.matmul(R_wc, c_camera_x_axis)
ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_x_axis[0], w_camera_x_axis[1], w_camera_x_axis[2], length=1, normalize=True, color='b')
#plot camera y axis in world frame
c_camera_y_axis = np.array([0, 1, 0])
w_camera_y_axis = np.matmul(R_wc, c_camera_y_axis)
ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_y_axis[0], w_camera_y_axis[1], w_camera_y_axis[2], length=1, normalize=True, color='g')


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

set_axes_equal(ax)

plt.show()
