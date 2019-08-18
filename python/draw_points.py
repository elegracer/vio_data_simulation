import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from GeometryLib import drawCoordinateFrame, euler2Rbn, euler2Rnb
import transformations as tf

x, y, z = [], [], []

with open('output/all_points.txt', 'r') as f:
    data = f.readlines()

    for line in data:
        odom = line.split(',')
        nums = list(map(float, odom))
        x.append(nums[0])
        y.append(nums[1])
        z.append(nums[2])

# print(len(x), x)
# print(len(y), y)
# print(len(z), z)

timestamp = []
position = []
quaterntions = []

with open('output/cam_pose.txt', 'r') as f:
    data = f.readlines()

    for line in data:
        odom = line.split(',')
        nums = list(map(float, odom))

        timestamp.append(nums[0])
        position.append([nums[1], nums[2], nums[3]])
        quaterntions.append(
            [nums[7], nums[4], nums[5], nums[6]])   # qw, qx, qy, qz

# plot 3d
fig = plt.figure()
plt.ion()
ax = fig.gca(projection='3d')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
rpy = []
t = []

for i in range(0, 400, 5):
    ax.clear()
    ax.scatter(x, y, z, c='g', label='points')

    x1, y1, z1 = [], [], []
    rpy.append(tf.euler_from_quaternion(quaterntions[i]))
    t.append(position[i])
    p = position[i]

    for j in range(len(rpy)):
        drawCoordinateFrame(ax, rpy[j], t[j])

    with open('output/all_points_' + str(i) + '.txt', 'r') as f:
        data = f.readlines()
        for line in data:
            odom = line.split(',')
            nums = list(map(float, odom))
            x1.append(nums[0])
            y1.append(nums[1])
            z1.append(nums[2])

            ax.plot([nums[0], p[0]], [nums[1], p[1]], zs=[nums[2], p[2]])

    with open('data/house.txt', 'r') as f:
        data = f.readlines()
        for line in data:
            odom = line.split()
            nums = list(map(float, odom))
            ax.plot([nums[0], nums[3]], [nums[1], nums[4]],
                    'b', zs=[nums[2], nums[5]])

    ax.scatter(x1, y1, z1, c='r', marker='^', label='trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-15, 20)
    ax.set_ylim(-15, 20)
    ax.set_zlim(0, 20)
    ax.legend()
    plt.show()
    plt.pause(0.001)

plt.show(block=True)
