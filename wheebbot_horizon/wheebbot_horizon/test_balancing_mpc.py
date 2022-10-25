#!/usr/bin/env python3

import argparse

# from balancing_utils import 

import subprocess

from termcolor import colored

import os

import shutil

from datetime import datetime
from datetime import date

from pathlib import Path

from balancing_utils.balancing_mpc import balancingMpc

from ament_index_python.packages import  get_package_share_path


script_dir = Path( __file__ ).parent.absolute()

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                    datetime.now().strftime("%H_%M_%S")

def main(args):

    stabilizer_test = balancingMpc(horizon_config_fullpath, \
                            actuators_config_fullpath, \
                            urdf_full_path, \
                            args.results_dir)

    stabilizer_test.init_prb()

if __name__ == '__main__':
    
    wheebbot_horizon_share_path = str(get_package_share_path('wheebbot_horizon'))
    wheebbot_urdf_share_path = str(get_package_share_path('wheebbot_description'))
    wheebbot_gazebo_share_path = str(get_package_share_path('wheebbot_gazebo'))
    # wheebbot_odrive_share_path = str(get_package_share_path('wheebbot_odrive'))
    wheebbot_odrive_share_path = "/home/andreap/wheebbot_ws/src/wheebbot-packages/wheebbot_odrive"
    parser = argparse.ArgumentParser(
        description='')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str, default=  wheebbot_urdf_share_path + "/urdf/" + "wheebbot_full.urdf")
    parser.add_argument('--results_dir', '-rdir', type = str, help = 'where results are saved', default = "/tmp/" + file_name + "_" + unique_id)
    parser.add_argument('--hor_confname', '-hconf', type = str,\
                        help = 'horizon config file name', default = file_name)

    args = parser.parse_args()

    urdfs_path = wheebbot_urdf_share_path + "/urdf"
    urdf_name = "wheebbot_full"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"
    
    config_path= wheebbot_horizon_share_path + "/config/" # configuration files path
    horizon_config_fullpath = config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = wheebbot_odrive_share_path + "/config/" + "actuators.yaml"

    floating_base_command = "is_floating_base:=" + "true"

    try:

        print(colored("\n--> GENERATING WHEEBBOT URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",\
                                        xacro_full_path, \
                                        floating_base_command, \
                                        "-o", 
                                        urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))

    os.mkdir(args.results_dir)
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "horizon_config.yaml" )
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro" )
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf" )

    main(args)