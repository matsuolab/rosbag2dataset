#!/usr/bin/env python3
import os
import argparse
import json
import glob
from tqdm import tqdm

import torch
import pickle

from rosbaghandler import RosbagHandler
from utils import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="hsr_config/hsr_dataset.json")
    args = parser.parse_args()

    if os.path.exists(args.config):
        with open(args.config, "r") as f:
            config = json.load(f)
    else:
        raise ValueError("cannot find config file")

    # os.chdir('../')
    bagfile_path = os.path.join(config["bagfile_dir"], config["bagfile_name"])
    print(bagfile_path)
    bagfiles = glob.glob(bagfile_path)
    if len(bagfiles) == 0:
        raise ValueError('set bagfile')
    # file_name = "test_traj"+str(config["traj_steps"])
    # out_dir = os.path.join(config["output_dir"], file_name)
    out_dir = config["output_dir"]
    print("out_dir: ", out_dir)
    os.makedirs(out_dir, exist_ok=True)

    torch_datasets = []
    file_name = ("test_1.pkl")
    torch_path = os.path.join(out_dir, file_name)
    for bagfile in bagfiles:
        rosbag_handler = RosbagHandler(bagfile)

        t0 = rosbag_handler.start_time
        t1 = rosbag_handler.end_time
        sample_data = rosbag_handler.read_messages(topics=config["topics"], start_time=t0, end_time=t1, hz=config["hz"])
        dataset = {}
        for topic in sample_data.keys():
            topic_type = rosbag_handler.get_topic_type(topic)
            print(topic)
            if topic_type == "sensor_msgs/Image" and topic == "hsrb/head_rgbd_sensor/rgb/image_raw":
                print("==== convert head image ====")
                dataset["head_images"] = convert_Image([sample_data[topic][0]], config["height"], config["width"])
            elif topic_type == "sensor_msgs/Image" and topic == "hsrb/hand_camera/image_raw":
                print("==== convert hand image ====")
                dataset["hand_images"] = convert_Image([sample_data[topic][0]], config["height"], config["width"])
            elif topic_type == "geometry_msgs/PoseStamped":
                print("==== convert PoseStamped ====")
                dataset["pose"] = convert_PoseStamped([sample_data[topic][0]])
            elif topic_type == "std_msgs/Float32":
                print("==== convert Float32 ====")
                dataset["gripper"] = [msg.data for msg in [sample_data[topic][0]]]
            elif topic_type == "sensor_msgs/JointState":
                dataset["observations"] = convert_JointStates([sample_data[topic][0]])
                # dataset["observations_all"] = convert_JointStates(sample_data[topic])

        print("==== save data as torch tensor ====")
        if "goal" in config["dataset"]:
            num_steps = len(dataset["head_images"]) - config["goal_steps"]
        else:
            num_steps = len(dataset["head_images"])

        num_traj = int(num_steps/config["traj_steps"])
        if num_traj == 0:
            num_traj = 1
            config["traj_steps"] = num_steps
        print(config)
        print(num_traj)
        for idx in tqdm(range(num_traj)):
            torch_dataset = {}
            t0 = idx*config["traj_steps"]
            t1 = t0+config["traj_steps"]
            for data_name in config["dataset"]:

                if data_name == "gripper":
                    traj_data = dataset[data_name][t0:t1]
                    torch_dataset["actions"] = torch.cat((torch_dataset["pose"], torch.tensor(traj_data, dtype=torch.float32).unsqueeze(-1)), 1)
                else:
                    traj_data = dataset[data_name][t0:t1]
                    torch_dataset[data_name] = torch.tensor(traj_data, dtype=torch.float32)
            torch_datasets.append(torch_dataset)

    with open(torch_path, "wb") as f:
        pickle.dump(torch_datasets, f)

        with open(os.path.join(out_dir, 'info.txt'), 'w') as f:
            info = config
            info['num_steps'] = num_steps
            info['num_traj'] = num_traj
            json.dump(config, f)