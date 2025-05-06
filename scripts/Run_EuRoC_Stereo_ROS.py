#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Run_EuRoC_Stereo_nonROS.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 09-19-2022
@version 1.0
@license Copyright (c) 2022
@desc None
"""


# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

DATA_ROOT = "/mnt/DATA/datasets/euroc/rosbags"
SeqNameList = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
]

# Result_root = os.path.join(os.environ["SLAM_RESULT"], "gf_orb_slam2/EuRoC/GFGG/")
Result_root = "/mnt/DATA/experiments/good_graph/openloop/laptop/rtabmap/"

# Number_GF_List = [400, 800, 1000, 1500]
Number_GF_List = [400]  #, 200]  # , 400]
NumRepeating = 1  # 10 # 20 #  5 #
SpeedPool = [1.0, 3.0]  # , 2.0, 3.0, 4.0, 5.0]  # x
SleepTime = 1  # 10 # 25
EnableViewer = False
EnableLogging = 1

# ----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


for speed in SpeedPool:

    Result_root_speed = Result_root + "_Speedx" + str(speed)

    for ri, num_gf in enumerate(Number_GF_List):

        Experiment_prefix = "ObsNumber_" + str(int(num_gf))

        for iteration in range(0, NumRepeating):

            Experiment_dir = os.path.join(Result_root_speed, Experiment_prefix + "_Round" + str(iteration + 1))

            # mkdir for pose traj
            cmd_mkdir = "mkdir -p " + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for sn, sname in enumerate(SeqNameList):

                SeqName = SeqNameList[sn]  # + '_blur_5'

                print(
                    bcolors.OKGREEN
                    + f"Seq: {SeqName}; Feature: {num_gf}; Speed: {speed}; Round: {iteration + 1}"
                    + bcolors().ENDC
                )

                file_traj = os.path.join(Experiment_dir, SeqName)
                file_log = "> " + file_traj + "_logging.txt" if EnableLogging else ""
                file_rosbag = os.path.join(DATA_ROOT, SeqName + ".bag")
                output_prefix = file_traj
                ratbmap_viz = "true" if EnableViewer else "false"

                # compose cmd
                cmd_slam = str(
                    "roslaunch rtabmap_odom euroc_datasets.launch"
                    + " output_prefix:="
                    + output_prefix
                    + " rtabmap_viz:="
                    + ratbmap_viz + " "
                    + file_log
                )

                cmd_rosbag = "rosbag play --clock " + file_rosbag + " -r " + str(speed)  # + ' -u 20'

                print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                subprocess.Popen(cmd_slam, shell=True)
                time.sleep(SleepTime * 3)

                print(bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC)
                proc_bag = subprocess.call(cmd_rosbag, shell=True)

                print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
                
                nodes = [
                    "/rtabmap/stereo_odometry",
                    "/rtabmap/rtabmap",
                    "/rtabmap/rtabmap_viz",
                    "/stereo_camera/stereo_image_proc",
                    "/stereo_camera/yaml_to_camera_info_left",
                    "/stereo_camera/yaml_to_camera_info_right",
                    "/cam0_imu_link",
                    "/cam1_imu_link",
                    "/imu_base_link",
                    "/imu_filter",
                ]

                for nodename in nodes:
                    subprocess.call(f"rosnode kill {nodename}", shell=True)
                    time.sleep(SleepTime * 3)
                # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                # time.sleep(15)
                time.sleep(SleepTime * 3)
