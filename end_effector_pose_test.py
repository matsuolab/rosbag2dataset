from mpl_toolkits.mplot3d import Axes3D
import torch
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
import glob

import matplotlib as mpl
mpl.use('tkagg')

folder = '/root/dataset'
names = sorted(glob.glob('{}/*'.format(folder), recursive=True))
end_effector_pose = torch.load('{}/end_effector_pose.pt'.format(names[0]))
pose_goal = torch.load('{}/pose_goal.pt'.format(names[0]))

print(end_effector_pose)

# x = end_effector_pose[:, 0]
# y = end_effector_pose[:, 1]
# z = end_effector_pose[:, 2]

# x = pose_goal[:, 0]
# y = pose_goal[:, 1]
# z = pose_goal[:, 2]

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')


def plot(ax, x, y, z, color='red'):
    ax.plot(x, y, z, color=color)
    ax.scatter(x, y, z, color=color)
    ax.scatter(x[0], y[0], z[0], color='blue')
    ax.scatter(x[-1], y[-1], z[-1], color='black')


plot(ax, end_effector_pose[:, 0], end_effector_pose[:, 1], end_effector_pose[:, 2], color='green')
plot(ax, pose_goal[:, 0], pose_goal[:, 1], pose_goal[:, 2], color='red')

ax.scatter([0], [0], [0])

# max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() * 0.5

# mid_x = (x.max()+x.min()) * 0.5
# mid_y = (y.max()+y.min()) * 0.5
# mid_z = (z.max()+z.min()) * 0.5
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()
# plt.savefig('end_effector_test.png')

# random_state = np.random.RandomState(0)
# ee2robot = pt.transform_from_pq(
#     np.hstack((np.array([0.4, -0.3, 0.5]),
#                pr.random_quaternion(random_state))))

# print(np.hstack((np.array([0.4, -0.3, 0.5]),
#                  pr.random_quaternion(random_state))))
