from mpl_toolkits.mplot3d import Axes3D
import torch
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
import glob

import matplotlib as mpl
mpl.use('tkagg')

def torch2numpy(x):
    return x.to('cpu').detach().numpy().copy()

folder = '/root/dataset'
names = sorted(glob.glob('{}/*'.format(folder), recursive=True))
end_effector_pose = torch2numpy(torch.load('{}/end_effector_pose.pt'.format(names[0])))
pose_goal = torch2numpy(torch.load('{}/pose_goal.pt'.format(names[0])))

print(end_effector_pose)

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

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()
# plt.savefig('end_effector_test.png')

