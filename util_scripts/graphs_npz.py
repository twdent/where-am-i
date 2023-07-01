# generate graphs for the ros_anymal/ETH_Cerberus_localization/num_matches_angle_seq.npz and
# ros_anymal/ETH_Cerberus_localization/num_matches_dist_seq.npz files

import numpy as np
import matplotlib.pyplot as plt

#load the data
num_matches_dist_seq = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_dist_seq.npz')['num_matches']
print(num_matches_dist_seq)
num_matches_angle_seq = np.load('./ros_anymal/ETH_Cerberus_localization/num_matches_angle_seq.npz')['num_matches']
print(num_matches_angle_seq)

#plot the data
plt.figure(1)
plt.plot(num_matches_dist_seq[:,0],num_matches_dist_seq[:,1],'-o')
plt.title('Number of matches vs. distance')
plt.xlabel('Distance')
plt.ylabel('Number of matches')
plt.savefig('num_matches_dist_seq.png')

plt.figure(2)
plt.plot(num_matches_angle_seq[:,0],num_matches_angle_seq[:,1],'-o')
plt.title('Number of matches vs. angle')
plt.xlabel('Angle')
plt.ylabel('Number of matches')
plt.savefig('num_matches_angle_seq.png')
plt.show()