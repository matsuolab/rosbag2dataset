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
    parser.add_argument('--config', type=str, default="config.json")
    args = parser.parse_args()

    if os.path.exists(args.config):
        with open(args.config, "r") as f:
            config = json.load(f)
    else:
        raise ValueError("cannot find config file")
    
    os.chdir('../')
    bagfile_path = os.path.join(config["bagfile_dir"], config["bagfile_name"]) 
    bagfiles = glob.glob(bagfile_path)
    print(bagfiles)
    if len(bagfiles) == 0:
        raise ValueError('set bagfile')
    # file_name = "test_traj"+str(config["traj_steps"])
    # out_dir = os.path.join(config["output_dir"], file_name)
    out_dir = config["output_dir"]
    print("out_dir: ", out_dir)
    os.makedirs(out_dir, exist_ok=True)

    torch_datasets = []
    file_name = ("test.pkl")
    torch_path = os.path.join(out_dir, file_name)
    for bagfile in [bagfiles[0]]:
        rosbag_handler = RosbagHandler(bagfile)
        t0 = rosbag_handler.start_time
        t1 = rosbag_handler.end_time
        sample_data = rosbag_handler.read_messages(topics=config["topics"], start_time=t0, end_time=t1, hz=config["hz"])
        for topic in sample_data.keys():
            topic_type = rosbag_handler.get_topic_type(topic)
            print(topic_type)
            if topic_type == "tf2_msgs/TFMessage":
                print("==== convert tf ====")
                data = convert_tf(sample_data[topic])
                print(data.shape)
        