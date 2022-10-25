#!/usr/bin/env python3

from ros2_utils import replay_trajectory

from horizon.utils import mat_storer

import argparse

import subprocess

import rospkg

from termcolor import colored

import casadi_kin_dyn

# resample solutions before replaying
refinement_scale = 10

def main(args):

    sliding_guide_command = "sliding_guide:=" + "true"

    try:

        print(colored("\n--> GENERATING LEG URDF...\n", "blue"))
        xacro_gen = subprocess.check_call(["xacro",\
                                        xacro_full_path, \
                                        sliding_guide_command, \
                                        "-o", 
                                        urdf_full_path])

        print(colored("\n--> URDF GENERATED SUCCESSFULLY. \n", "blue"))

    except:

        print(colored('FAILED TO GENERATE URDF.', "red"))

    try:

        rviz_window = subprocess.Popen(["roslaunch",\
                                        "awesome_leg",\
                                        "horizon_rviz.launch", \
                                        sliding_guide_command])

    except:

        print('Failed to launch RViz.')

    ms_loader = mat_storer.matStorer(args.replay_path + "/" + args.replay_filename + ".mat")
    loaded_sol = ms_loader.load()
    q_p = loaded_sol["q_p"]
    # q_dot = loaded_sol["q_p_dot"]
    # q_ddot = loaded_sol["q_p_ddot"]
    f_contact = loaded_sol["f_contact"]

    sol_contact_map = dict()
    sol_contact_map["tip1"] = f_contact

    dt_opt = loaded_sol["dt_opt"][0][0]
    
    urdf = open(urdf_full_path, "r").read()
    kin_dyn_model = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf)

    joint_names = kin_dyn_model.joint_names()
    # joint_names.remove("universe")  # removing the "universe joint"

    rpl_traj = replay_trajectory(dt_opt, joint_names, q_p, sol_contact_map, \
                cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kin_dyn_model)  # replaying the (resampled) trajectory

    rpl_traj.sleep(1.)
    rpl_traj.replay(is_floating_base = False)

if __name__ == '__main__':

    # useful paths
    rospackage = rospkg.RosPack() # Only for taking the path to the leg package

    # adding script arguments
    parser = argparse.ArgumentParser(
        description='just a simple test file for RePAIR co-design')
    parser.add_argument('--replay_path', '-path', type=str, default = rospackage.get_path("awesome_leg") + \
                        "/opt_results/horizon_jump/replay_directory")
    parser.add_argument('--replay_filename', '-fname', default ='awesome_jump_res', type=str)
    args = parser.parse_args()

    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code"
    urdfs_path = rospackage.get_path("awesome_leg") + "/description/urdf"
    urdf_name = "awesome_leg_test_rig"
    urdf_full_path = urdfs_path + "/" + urdf_name + ".urdf"
    xacro_full_path = urdfs_path + "/" + urdf_name + ".urdf.xacro"

    main(args)