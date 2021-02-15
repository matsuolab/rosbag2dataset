from os.path import join
import numpy as np
import cv2
from tqdm import tqdm
from geometry_msgs.msg import Vector3
import tf
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import *
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager


def convert_Image(data, height=None, width=None):
    obs = []
    bridge = CvBridge()
    for msg in tqdm(data):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            if height is not None and width is not None:
                h, w, c = img.shape
                img = img[0:h, int((w - h) * 0.5):w - int((w - h) * 0.5), :]
                img = cv2.resize(img, (height, width))
            obs.append(img)
        except CvBridgeError as e:
            print(e)
    return obs


def convert_CompressedImage(data, height=None, width=None):
    obs = []
    for msg in tqdm(data):
        img = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if height is not None and width is not None:
            h, w, c = img.shape
            img = img[0:h, int((w - h) * 0.5):w - int((w - h) * 0.5), :]
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


def convert_Twist(data, action_noise, lower_bound, upper_bound):
    acs = []
    for msg in tqdm(data):
        # action
        vel = np.array([msg.linear.x, msg.angular.z])
        vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
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
    trans_pose = np.array([x * np.cos(base_pose[2]) + y * np.sin(base_pose[2]),
                           -x * np.sin(base_pose[2]) +
                           y * np.cos(base_pose[2]),
                           np.arctan2(np.sin(yaw), np.cos(yaw))])
    return trans_pose


def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])


def angle_normalize(z):
    return np.arctan2(np.sin(z), np.cos(z))


def get_pose_from_odom(odom):
    yaw = quaternion_to_euler(odom.pose.pose.orientation).z
    pose = np.array([odom.pose.pose.position.x,
                     odom.pose.pose.position.y, yaw])
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


def convert_JointStates(data):
    joint_states_list = []
    for msg in tqdm(data):
        position = np.array(msg.position)
        velocity = np.array(msg.velocity)
        joint_states = np.concatenate([position, velocity])
        joint_states_list.append(joint_states)
    return joint_states_list


def convert_JointTrajectory(data):
    joint_trajectory_list = []
    for msg in tqdm(data):
        points = msg.points[0]
        position = np.array(points.positions)
        velocity = np.array(points.velocities)
        acceleration = np.array(points.accelerations)
        joint_trajectory = np.concatenate([position, velocity, acceleration])
        joint_trajectory_list.append(joint_trajectory)
    return joint_trajectory_list


def convert_tf(data):
    tf_list = []
    for msg in tqdm(data):
        tm = TransformManager()
        tf_link = pt.transform_from_pq([0, 0, 0, 0, 0, 0, 0])
        tm.add_transform('link0', 'robot', tf_link)

        for tf in msg.transforms:
            for i in range(1, 8):
                if tf.child_frame_id == 'link{}'.format(i):
                    trans = tf.transform.translation
                    quat = tf.transform.rotation
                    tf_link = pt.transform_from_pq(
                        [trans.x, trans.y, trans.z, quat.x, quat.y, quat.z, quat.w])
                    tm.add_transform('link{}'.format(i),
                                     'link{}'.format(i-1), tf_link)

        end_effector_matrix = tm.get_transform('link7', 'link0')
        pos = end_effector_matrix[:3, 3]
        end_effector_rotation_matrix = end_effector_matrix[:3, :3]
        rot = Rotation.from_matrix(
            end_effector_rotation_matrix).as_euler('xyz')
        end_effector_pose = np.concatenate([pos, rot])
        tf_list.append(end_effector_pose)

    return tf_list


def convert_GripperFeedback(data):
    gripper_feedback_list = []
    for msg in tqdm(data):
        gripper_feedback_list.append(msg.feedback.current_pulse)
    return gripper_feedback_list


def convert_GripperGoal(data):
    gripper_goal_list = []
    for msg in tqdm(data):
        gripper_goal_list.append(np.array([
            msg.goal.target_pulse,
            msg.goal.pulse_speed
        ]))
    return gripper_goal_list
