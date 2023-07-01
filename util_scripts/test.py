import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import os

OBJ_CENTRE_ECEF = np.array([4279256.63, 643234.64, 4670497.61])
SCALE_OBJ_TO_WORLD = 26.5

#graph the stuff in the ros_anymal/ETH_Cerberus_localization/positions.npz file
#saved as follows:
# np.savez('./ros_anymal/ETH_Cerberus_localization/positions.npz',
#                 xy_lio_odom = np.array(xy_lio_odom),
#                 xy_state_odom = np.array(xy_state_odom),
#                 xy_rtk = np.array(xy_rtk)
#                 )
#load the data
xy_pnp_pos = np.load('./ros_anymal/ETH_Cerberus_localization/xy_pnp_pos.npz')
xy_pnp_pos = xy_pnp_pos['xy_pnp_pos']

#scale to world
xy_pnp_pos = xy_pnp_pos * SCALE_OBJ_TO_WORLD
xy_pnp_pos = xy_pnp_pos[3:, :]

model_path = os.path.expanduser("~")+"/Documents/models3D/obj/ETHmodel.obj"
mesh = o3d.io.read_triangle_mesh(model_path)
mesh.scale(SCALE_OBJ_TO_WORLD, center=np.array([0,0,0]))
pts = np.asarray(mesh.vertices)[0::50]
#plot
plt.figure(2)

plt.plot(xy_pnp_pos[:,0], xy_pnp_pos[:,1], '-o')
#plot some model xy points
plt.scatter(pts[:, 0], pts[:, 1], c='k', marker='.')
plt.xlabel('x')
plt.ylabel('y')

#equal axis scaling
plt.gca().set_aspect('equal', adjustable='box')

# plt.savefig('./ros_anymal/ETH_Cerberus_localization/xy_pnp_pos.png')

plt.show()


# #load the position data
# # xy_lio_odom = np.load('./ros_anymal/ETH_Cerberus_localization/positions.npz')['xy_lio_odom']
# xy_lio_odom = np.load('./ros_anymal/ETH_Cerberus_localization/odometry.npz')['compslam_lio_odom'][3:,:]
# #rotate angle degrees ccw to match the pnp data
# angle = 35
# rad_ang = np.deg2rad(angle)
# xy_lio_odom = np.array([xy_lio_odom[:,0]*np.cos(rad_ang) - xy_lio_odom[:,1]*np.sin(rad_ang),
#                         xy_lio_odom[:,0]*np.sin(rad_ang) + xy_lio_odom[:,1]*np.cos(rad_ang)]).T
# #shift to match the pnp data
# xy_lio_odom[:,0] = xy_lio_odom[:,0] - 155
# xy_lio_odom[:,1] = xy_lio_odom[:,1] + 195


# xy_state_odom = np.load('./ros_anymal/ETH_Cerberus_localization/positions.npz')['xy_state_odom']
# xy_rtk = np.load('./ros_anymal/ETH_Cerberus_localization/positions.npz')['xy_rtk']

# #center the rtk data on x and y axes
# xy_rtk[:,0] = xy_rtk[:,0] - OBJ_CENTRE_ECEF[0]
# xy_rtk[:,1] = xy_rtk[:,1] - OBJ_CENTRE_ECEF[1]

# #plot
# plt.figure(1)
# plt.plot(xy_lio_odom[:,0], xy_lio_odom[:,1], '-', label='compslam_lio_odom')
# # plt.plot(xy_state_odom[:,0], xy_state_odom[:,1], '-', label='state_est_odom')
# # plt.plot(xy_rtk[:,1], -xy_rtk[:,0], '-', label='rtk')
# plt.plot(xy_pnp_pos[:,0], xy_pnp_pos[:,1], '-', label='pnp')

# plt.scatter(pts[:, 0], pts[:, 1], c='k', marker='.')
# plt.xlabel('x')
# plt.ylabel('y')
# #set axis equal
# plt.gca().set_aspect('equal', adjustable='box')
# plt.legend()


# plt.show()