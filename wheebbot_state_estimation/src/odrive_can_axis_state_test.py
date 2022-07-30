#!/usr/bin/env python3

import can
import cantools
import os

def main():

    PACKAGE_DIR = os.path.normpath(os.getcwd() + os.sep + os.pardir)
    print(PACKAGE_DIR)
    DBC_PATH = os.path.join(PACKAGE_DIR, 'config/CAN_config', 'imu_can.dbc')

    ODrv_db = cantools.database.load_file(DBC_PATH)

    socketcan_bus_name="can0"
    bus = can.Bus(socketcan_bus_name, bustype="socketcan")
    print("\n Receiving on " + socketcan_bus_name + "bus")


    axisID_left = 0x0 # axis ID on the left wheel (seeing the robot from behind)
    axisID_right = 0x1 # axis ID on the right wheel (seeing the robot from behind)

    #### IMPORTANT ####
    # To make motor axes rotation coherent with the IMU and robot ref frame,
    # left axis readings need to be inverted, while right axis ones are ok.

    # Read messages infinitely and wait for the right ID to show up
    while True:
        # Read messages infinitely and wait for the right ID to show up
        msg = bus.recv()

        if msg.arbitration_id == ((axisID_left << 5) | ODrv_db.get_message_by_name('Get_Encoder_Estimates').frame_id): # from left axis

            lft_ax_pos = -ODrv_db.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
            lft_ax_vel = -ODrv_db.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']

            print("Left axis:\n ")
            print("Position: "+ str(lft_ax_pos)+"\t[turns]"+"\n")
            print("Velocity: "+ str(lft_ax_vel)+"\t[turns/s]"+"\n")
        
        elif msg.arbitration_id == ((axisID_right << 5) | ODrv_db.get_message_by_name('Get_Encoder_Estimates').frame_id): # from right axis

            rght_ax_pos = ODrv_db.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
            rght_ax_vel = ODrv_db.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']

            print("Right axis:\n ")
            print("Position: "+ str(rght_ax_pos)+"\t[turns]"+"\n")
            print("Velocity: "+ str(rght_ax_vel)+"\t[turns/s]"+"\n")


if __name__ == '__main__':

    main()