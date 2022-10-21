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

script_dir = Path( __file__ ).parent.absolute()

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" +\
                    datetime.now().strftime("%H_%M_%S")

def main(args):

    stabilizer = balancingMpc()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description='')

    # first level specific arguments
    parser.add_argument('--urdf_path', '-urdf', type = str, default=  script_dir + "../../wheebbot_description/urdf"+ "/" + "wheebbot.urdf")
    parser.add_argument('--results_dir', '-rdir', type = str, help = 'where results are saved', default = "/tmp/" + file_name + "_" + unique_id)
    parser.add_argument('--hor_confname', '-hconf', type = str,\
                        help = 'horizon config file name', default = file_name)

    args = parser.parse_args()

    metapackage_path = script_dir + "/../../../"

    urdfs_path = metapackage_path + "wheebbot_description/urdf"
    urdf_name = "wheebbot"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"
    
    config_path=metapackage_path + "wheebbot_horizon" + "/config/" # configuration files path
    horizon_config_fullpath = config_path + args.hor_confname + ".yaml"
    actuators_config_fullpath = config_path + "actuators.yaml"

    os.mkdir(args.results_dir)
    shutil.copyfile(horizon_config_fullpath, args.results_dir + "/" + "horizon_config.yaml" )
    shutil.copyfile(xacro_full_path, args.results_dir + "/" + urdf_name + ".urdf.xacro" )
    shutil.copyfile(urdf_full_path, args.results_dir + "/" + urdf_name + ".urdf" )

    try:

        print(colored("\n--> GENERATING WHEEBBOT URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",\
                                        xacro_full_path, \
                                        "-o", 
                                        urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))

    main(args)