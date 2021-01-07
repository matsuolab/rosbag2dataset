#!/usr/bin/env python3

import glob
import os
import argparse
import json
from tqdm import tqdm

import torch

from rosbaghandler import RosbagHandler
from utils import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="config.json")
    parser.add_argument('--skip_if_exists', action='store_true')
    args = parser.parse_args()

    if os.path.exists(args.config):
        with open(args.config, "r") as f:
            config = json.load(f)
    else:
        raise ValueError("cannot find config file")
    if config["bagfile_names"]:
        bagfile_names = config["bagfile_names"]
    else:
        # convert all .bag file if bagfile_names is not specified
        bagfile_names = [os.path.basename(r) for r in glob.glob(config["bagfile_dir"]+'/*.bag')]
    print('Converting {} files'.format(len(bagfile_names)))

    # TODO: Refactor
    for bagfile_name in bagfile_names:
        bagfile = os.path.join(config["bagfile_dir"], bagfile_name) 
        if not os.path.exists(bagfile):
            raise ValueError('set correct bagfiles')
        file_name = os.path.splitext(os.path.basename(bagfile))[0]+"_"+str(config["hz"])+"hz"
        out_dir = os.path.join(config["output_dir"], file_name)
        print("out_dir: ", out_dir)
        os.makedirs(out_dir, exist_ok=True)
        rosbag_handler = RosbagHandler(bagfile)

        t0 = rosbag_handler.start_time
        t1 = rosbag_handler.end_time
        topics = config["topics"].keys()
        sample_data = rosbag_handler.read_messages(topics=topics, start_time=t0, end_time=t1, hz=config["hz"])
        for topic in topics:
            data_name = config["topics"][topic]
            topic_type = rosbag_handler.get_topic_type(topic)
            pt_name = data_name + ".pt"
            path = os.path.join(out_dir, pt_name)
            if os.path.exists(path) and args.skip_if_exists:
                print("{} already exists. Skipping.".format(pt_name))
                continue
            if topic_type == "sensor_msgs/CompressedImage":
                print("==== convert compressed image ====")
                data = convert_CompressedImage(sample_data[topic], config["height"], config["width"])
                tensor = torch.tensor(data, dtype=torch.float32).permute(0, 3, 1, 2) 
            elif topic_type == "sensor_msgs/Image":
                print("==== convert image ====")
                data = convert_Image(sample_data[topic], config["height"], config["width"])
                tensor = torch.tensor(data, dtype=torch.uint8).permute(0, 3, 1, 2)
            elif topic_type == "geometry_msgs/PoseStamped":
                print("==== convert PoseStamped ====")
                data = convert_PoseStamped(sample_data[topic])
                tensor = torch.tensor(data, dtype=torch.float32)
            elif topic_type == "sensor_msgs/JointState":
                print("==== convert JointState ====")
                data = convert_JointStates(sample_data[topic])
                tensor = torch.tensor(data, dtype=torch.float32)
            elif topic_type == "trajectory_msgs/JointTrajectory":
                print("==== convert JointTrajectory ====")
                data = convert_JointTrajectory(sample_data[topic])
                tensor = torch.tensor(data, dtype=torch.float32)
            elif topic_type == "std_msgs/Float32":
                print("==== convert Float32 ====")
                data = [msg.data for msg in sample_data[topic]]
                tensor = torch.tensor(data, dtype=torch.float32) 
            else:
                tensor = None
            if tensor is not None:
                print("==== save {} as torch tensor {} ({})====".format(topic, pt_name, tensor.dtype))
                with open(path, "wb") as f:
                    torch.save(tensor, f)
        with open(os.path.join(out_dir, 'info.json'), 'w') as f:
            info = config
            json.dump(info, f)
