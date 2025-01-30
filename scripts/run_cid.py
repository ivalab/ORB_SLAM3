#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_cid.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-18-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


from pathlib import Path
import subprocess
from dataclasses import dataclass
import time


@dataclass
class SeqMetaData:
    name: str
    prefix: str
    layout: str  # camera position: high or low


@dataclass
class MethodMetaData:
    name: str
    feature_num: int
    sensor: str


DATA_ROOT = Path("/mnt/DATA/datasets/cid")
OUTPUT_ROOT = Path("/mnt/DATA/experiments/semantic/cid/covis/medium/50/")
SEQUENCES = [
    SeqMetaData("floor13_1", "", "high"),
    # SeqMetaData("office_1", "", "high"),
    # SeqMetaData("apartment1_1", "", "low"),
]
METHODS = [
    # MethodMetaData("orb3_rgbd", 1000, "cam"),
    # MethodMetaData("orb3_rgbd_wo", 1000, "wo"),
    MethodMetaData("orb3_rgbd_imu", 1000, "imu"),
]

SPEEDS = [1.0]  # , 0.1]  # , 2.0, 3.0]
ROUNDS = 1
ENABLE_VIEWER = 1
ENABLE_LOGGING = 0

PROJECT_PATH = Path(__file__).resolve().parent.parent

for speed_idx, speed in enumerate(SPEEDS):
    for seq_idx, seq in enumerate(SEQUENCES):
        for method_idx, method in enumerate(METHODS):
            for round_idx in range(1, ROUNDS + 1):
                method_dir = (
                    OUTPUT_ROOT / f"{method.name}/_Speedx{speed}/ObsNumber_{method.feature_num}_Round{round_idx}"
                )
                method_dir.mkdir(parents=True, exist_ok=True)
                print(
                    f"Seq: {seq.name}, Method: {method.name}, Feature: {method.feature_num}, Speed: {speed}, Round: {round_idx}"
                )

                voc_file = PROJECT_PATH / "Vocabulary/ORBvoc.txt"
                yaml_file = PROJECT_PATH / "Examples/Data/cid/cid_inertial.yaml"
                if "low" == seq.layout:
                    yaml_file = PROJECT_PATH / "Examples/Data/cid/cid_inertial_low.yaml"
                asso_file = PROJECT_PATH / f"Examples/Data/cid/{seq.name}_associated.txt"
                seq_dir = DATA_ROOT / seq.name
                imu_file = seq_dir / "imu.txt"
                wo_file = seq_dir / "odom_corrected.txt"
                bin_file = PROJECT_PATH / "Examples/RGB-D/rgbd_tum"
                cmd = f"{bin_file} -v {voc_file} -y {yaml_file} -z {ENABLE_VIEWER} -s {seq_dir} -a {asso_file} -n {seq.name} -o {method_dir}"

                if "imu" in method.sensor:
                    cmd += f" -i {imu_file}"
                elif "wo" in method.sensor:
                    cmd += f" -w {wo_file}"

                if ENABLE_LOGGING:
                    cmd += f' > {method_dir / (seq.name + "_log.txt")}'

                print(cmd)

                subprocess.call(cmd, shell=True)
                time.sleep(1)

                # ../Examples/RGB-D/rgbd_tum -v ../Vocabulary/ORBvoc.txt -s ../Examples/Data/cid/cid_inertial.yaml -q /mnt/DATA/datasets/cid/office_1/ -a ../Examples/Data/cid/office_1_associated.txt -t office_1 -o /mnt/DATA/experiments/semantic/cid/orb3/ -i /mnt/DATA/datasets/cid/office_1/imu.txt
