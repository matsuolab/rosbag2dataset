import numpy as np
import cv2
from tqdm import tqdm
from geometry_msgs.msg import Vector3
import tf
from tf.transformations import *
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge, CvBridgeError
import os
import yaml

import rospy
import rospkg

import tf2_ros
import geometry_msgs.msg
from std_srvs.srv import Empty, EmptyResponse
#from pytransform3d.transform_manager import TransformManager
import matplotlib.pyplot as plt

def convert_Image(data, height=None, width=None):
    obs = []
    bridge = CvBridge()
    for msg in tqdm(data):
        """ try:
            img = bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e) """
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if height is not None and width is not None:
            h,w,c = img.shape
            img = img[0:h, int((w-h)*0.5):w-int((w-h)*0.5), :]
            img = cv2.resize(img, (height, width))
        obs.append(img)
    return obs

def convert_CompressedImage(data, height=None, width=None):
    obs = []
    for msg in tqdm(data):
        img = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if height is not None and width is not None:
            h,w,c = img.shape
            img = img[0:h, int((w-h)*0.5):w-int((w-h)*0.5), :]
            img = cv2.resize(img, (height, width))
        obs.append(img)
    return obs

def convert_Odometry(data, action_noise, lower_bound, upper_bound):
    acs = []
    pos = []
    goals = []
    for msg in tqdm(data):
        # action
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
        acs.append(vel)
        # pose
        pose = get_pose_from_odom(msg)
        pos.append(pose)
    return acs, pos

def convert_Twist(data, action_noise=0.1, lower_bound=[0.0, -1.5], upper_bound=[1.5, 1.5]):
    acs = []
    for msg in tqdm(data):
        # action
        vel = np.array([msg.linear.x, msg.angular.z])
        # vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
        acs.append(vel)
    return acs

def convert_LaserScan(data):
    lidar = []
    for msg in tqdm(data):
        lidar.append(np.array(msg.ranges))
    return lidar

def convert_Imu(data):
    imu = []
    for msg in tqdm(data):
        # imu
        imu_data = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z])
        imu.append(imu_data)
    return imu

def transform_pose(pose, base_pose):
    x = pose[0] - base_pose[0]
    y = pose[1] - base_pose[1]
    yaw = pose[2] - base_pose[2]
    trans_pose = np.array([ x*np.cos(base_pose[2]) + y*np.sin(base_pose[2]),
                           -x*np.sin(base_pose[2]) + y*np.cos(base_pose[2]),
                           np.arctan2(np.sin(yaw), np.cos(yaw))])
    return trans_pose

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def angle_normalize(z):
    return np.arctan2(np.sin(z), np.cos(z))

def get_pose_from_odom(odom):
    yaw = quaternion_to_euler(odom.pose.pose.orientation).z
    pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    return pose

def add_random_noise(action, std, lb, ub):
    action += np.random.randn(*action.shape) * std
    return action.clip(lb, ub)

def convert_PoseStamped(data):
    pose_list = []
    for msg in tqdm(data):
        quaternion = msg.pose.orientation
        euler = quaternion_to_euler(quaternion)

        pose_list.append(np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            euler.x,
            euler.y,
            euler.z,
        ]))
    return pose_list

class GetEndeffPos:
    def __init__(self):
        # Is node_name 'saving_calibration_result' overlap a problem or not?
        rospy.init_node("get_endeff_pos")
        

        # parent frame
        self.parent_frame_id = "world"
        
        # child frame
        self.child_frame_id = "link7"

        # tf buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rospy.Time())
                print(trans.transform.translation)
            except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)
        return EmptyResponse()

def transform2homogeneousM(tfobj):
    # geometry_msg.msg.Transform to Homogeneous Matrix
    tfeul = tf.transformations.euler_from_quaternion(
        [tfobj.rotation.x, tfobj.rotation.y, tfobj.rotation.z, tfobj.rotation.w], axes='sxyz')
    tftrans = [tfobj.translation.x, tfobj.translation.y, tfobj.translation.z]
    tfobjM = tf.transformations.compose_matrix(angles=tfeul, translate=tftrans)
    return tfobjM

def convert_tf(data):
    tf_list = []
    for msg in tqdm(data):
        print(msg)
        # Initialize Homogeneous Matrix
        tfobj = geometry_msgs.msg.Transform()
        tf_target = transform2homogeneousM(tfobj)
        # define frame start & end
        frame_start = "world"
        frame_end = "link7"

        for tfobj in msg.transforms:
            if tfobj.header.frame_id == frame_start:
                tfobjM = transform2homogeneousM(tfobj.transform)
                tf_target = tf_target.dot(tfobjM)
                if tfobj.child_frame_id == frame_end:
                    break
                else:
                    frame_start = tfobj.child_frame_id
        print(tf_target)
        pos = tf_target[:3, 3]
        # print(pos)
        rot = Rotation.from_matrix(tf_target[:3, :3]).as_euler('xyz')
        end_effector_pose = np.concatenate([pos, rot])
        tf_list.append(end_effector_pose)
        # print(end_effector_pose)

    tf_list = np.array(tf_list)
    fig = plt.figure(figsize = (8, 8))

    ax = fig.add_subplot(111)

    ax.set_title("", size = 20)

    ax.set_xlabel("x", size = 14, color = "r")
    ax.set_ylabel("y", size = 14, color = "r")

    ax.scatter(tf_list[:, 0], tf_list[:, 1], s = 40, c = "blue")
    
    os.chdir('rosbag2dataset')
    plt.savefig('tf_xy.png')
    plt.close()

    fig = plt.figure(figsize = (8, 8))

    ax = fig.add_subplot(111)

    ax.set_title("", size = 20)

    ax.set_xlabel("y", size = 14, color = "r")
    ax.set_ylabel("z", size = 14, color = "r")

    ax.scatter(tf_list[:, 1], tf_list[:, 2], s = 40, c = "blue")
    
    plt.savefig('tf_yz.png')
    plt.close()

    return tf_list

def convert_EndEffectorPose(data):
    end_effector_pose_list = []
    for msg in tqdm(data):
        # print(msg)
        end_effector_pose_list.append([msg.pose[0] * 0.001, msg.pose[1] * 0.001, msg.pose[2] * 0.001])

    return np.array(end_effector_pose_list)

def convert_JointStates(data, all=False):
    joint_states_list = []
    tmp_data = data[0]
    joint_names = tmp_data.name
    for msg in tqdm(data):
        joint_states_list.append([msg.position, msg.velocity, msg.effort])

    return np.array(joint_states_list)
    
