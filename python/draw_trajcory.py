import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

positions = []

with open('output/imu_pose.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split(',')
        nums = list(map(float, odom))

        positions.append([nums[1], nums[2], nums[3]])

positions_int = []

with open('output/imu_int_pose.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split(',')
        nums = list(map(float, odom))

        positions_int.append([nums[1], nums[2], nums[3]])

positions_int_noisy = []

with open('output/imu_int_pose_noisy.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split(',')
        nums = list(map(float, odom))

        positions_int_noisy.append([nums[1], nums[2], nums[3]])

# plot 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot(*zip(*positions), label='gt')
ax.plot(*zip(*positions_int), label='imu_int')
# ax.plot(*zip(*positions_int_noisy), label='imu_int_noisy')
ax.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
