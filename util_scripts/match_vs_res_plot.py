import numpy as np
import matplotlib.pyplot as plt

#compare matches count for different resolutions
num_matches_640x480 = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_640x480.npz')
num_matches_640x480_rendHD = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_640x480_rendHD.npz')
# num_matches_640x480_rendHD2 = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_640x480_rendHD2.npz')
# num_matches_720x540 = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_720x540.npz')
num_matches_1440x1080 = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_1440x1080.npz') 

plt.figure(1)
plt.plot(num_matches_640x480['num_matches'], '-o', label='Ego: 640x480, Render: 640x480')
plt.plot(num_matches_640x480_rendHD['num_matches'], '-o', label='Ego: 640x480, Render: 1440x1080')
# plt.plot(num_matches_640x480_rendHD2['num_matches'], '-o', label='640x480_rendHD2')
# plt.plot(num_matches_720x540['num_matches'], '-o', label='720x540')
plt.plot(num_matches_1440x1080['num_matches'], '-o', label='Ego: 1440x1080, Render: 1440x1080')
plt.xlabel('Frame')
plt.ylabel('Match count')
plt.legend()
plt.show()
