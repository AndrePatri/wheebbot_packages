#!/usr/bin/env python3

import can
import cantools
import os

def main():

    PACKAGE_DIR = os.path.normpath(os.getcwd() + os.sep + os.pardir)
    DBC_PATH = os.path.join(PACKAGE_DIR, 'config/CAN_config', 'imu_can.dbc')

    IMU_db = cantools.database.load_file(DBC_PATH)
    # print(IMU_db)

    socketcan_bus_name =  "can1"
    bus = can.Bus(socketcan_bus_name, bustype = "socketcan")

    print("\n Receiving on " + socketcan_bus_name + "bus")

    # Read messages infinitely and wait for the right ID to show up
    while True:
        msg = bus.recv()

        # HEARTBEAT
        if msg.arbitration_id == (IMU_db.get_message_by_name('imu_Heartbeat').frame_id):

            orientCalStatus085 = IMU_db.decode_message('imu_Heartbeat', msg.data)['orient_cal_status085']
            gyroCalStatus055 = IMU_db.decode_message('imu_Heartbeat', msg.data)['orient_cal_status055']
            imuTemp = IMU_db.decode_message('imu_Heartbeat', msg.data)['imu_temp']

            print("BNO085 orientation calibration status (0-3): "+ str(orientCalStatus085)+"\n")
            print("BNO055 gyroscope calibration status (0-3): "+ str(gyroCalStatus055)+"\n")
            print("Sensor temperature: "+ str(imuTemp)+"°C\n")

        # ORIENTATION QUATERNION
        if msg.arbitration_id == (IMU_db.get_message_by_name('quat_First_Half').frame_id):
            q_w = IMU_db.decode_message('quat_First_Half', msg.data)['q_w']
            q_i = IMU_db.decode_message('quat_First_Half', msg.data)['q_i']
            print("q_w:\t"+ str(q_w)+"\n")
            print("q_i:\t"+ str(q_i)+"\n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('quat_Second_Half').frame_id):
            q_j = IMU_db.decode_message('quat_Second_Half', msg.data)['q_j']
            q_k = IMU_db.decode_message('quat_Second_Half', msg.data)['q_k']
            print("q_j:\t"+ str(q_j)+"\n")
            print("q_k:\t"+ str(q_k)+"\n")

        # EULER YPR
        if msg.arbitration_id == (IMU_db.get_message_by_name('Euler_First_Half').frame_id):
            Yaw = IMU_db.decode_message('Euler_First_Half', msg.data)['Euler_Yaw']
            Pitch = IMU_db.decode_message('Euler_First_Half', msg.data)['Euler_Pitch']
            print("Yaw:\t"+ str(Yaw)+"\t rad\n")
            print("Pitch:\t"+ str(Pitch)+"\t rad\n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Euler_Second_Half').frame_id):
            ROll = IMU_db.decode_message('Euler_Second_Half', msg.data)['Euler_Roll']
            IsDeg = IMU_db.decode_message('Euler_Second_Half', msg.data)['Is_Deg']
            print("Roll:\t"+ str(ROll)+"\t rad \n")
            print("IsDeg:\t"+ str(IsDeg)+"\n")

        # GYRO
        if msg.arbitration_id == (IMU_db.get_message_by_name('Gyro_First_Half').frame_id):
            GyroX = IMU_db.decode_message('Gyro_First_Half', msg.data)['gyro_x']
            GyroY = IMU_db.decode_message('Gyro_First_Half', msg.data)['gyro_y']
            print("GyroX:\t"+ str(GyroX)+"\t rad/s \n")
            print("GyroY:\t"+ str(GyroY)+"\t rad/s \n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Gyro_Second_Half').frame_id):
            GyroZ = IMU_db.decode_message('Gyro_Second_Half', msg.data)['gyro_z']
            print("GyroZ:\t"+ str(GyroZ)+"\t rad/s \n")

        # LIN ACC
        if msg.arbitration_id == (IMU_db.get_message_by_name('Linear_Acc_First_Half').frame_id):
            Xddot = IMU_db.decode_message('Linear_Acc_First_Half', msg.data)['x_ddot']
            Yddot = IMU_db.decode_message('Linear_Acc_First_Half', msg.data)['y_ddot']
            print("x_ddot:\t"+ str(Xddot)+"\t m/s^2 \n")
            print("y_ddot:\t"+ str(Yddot)+"\t m/s^2 \n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Linear_Acc_Second_Half').frame_id):
            Zddot = IMU_db.decode_message('Linear_Acc_Second_Half', msg.data)['z_ddot']

            print("z_ddot:\t"+ str(Zddot)+"\t m/s^2 \n")

        # GRAVITY
        if msg.arbitration_id == (IMU_db.get_message_by_name('Grav_First_Half').frame_id):
            gX = IMU_db.decode_message('Grav_First_Half', msg.data)['g_x']
            gY = IMU_db.decode_message('Grav_First_Half', msg.data)['g_y']
            print("g_x:\t"+ str(gX)+"\t m/s^2 \n")
            print("g_y:\t"+ str(gY)+"\t m/s^2 \n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Grav_Second_Half').frame_id):
            gZ = IMU_db.decode_message('Grav_Second_Half', msg.data)['g_z']
            print("g_z:\t"+ str(gZ)+"\t m/s^2 \n")

        # RAW GYRO
        if msg.arbitration_id == (IMU_db.get_message_by_name('Raw_Gyro_First_Half').frame_id):
            rawGyroX = IMU_db.decode_message('Raw_Gyro_First_Half', msg.data)['raw_gyro_x']
            rawGyroY = IMU_db.decode_message('Raw_Gyro_First_Half', msg.data)['raw_gyro_y']
            print("GyroX(raw):\t"+ str(rawGyroX)+"\t rad/s \n")
            print("GyroY (raw):\t"+ str(rawGyroY)+"\t rad/s \n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Raw_Gyro_Second_Half').frame_id):
            rawGyroZ = IMU_db.decode_message('Raw_Gyro_Second_Half', msg.data)['raw_gyro_z']
            print("GyroZ (raw):\t"+ str(rawGyroZ)+"\t rad/s \n")

        # RAW ACC
        if msg.arbitration_id == (IMU_db.get_message_by_name('Raw_Acc_First_Half').frame_id):
            rawXddot = IMU_db.decode_message('Raw_Acc_First_Half', msg.data)['raw_x_ddot']
            rawYddot = IMU_db.decode_message('Raw_Acc_First_Half', msg.data)['raw_y_ddot']
            print("x_ddot (raw):\t"+ str(rawXddot)+"\t m/s^2 \n")
            print("y_ddot (raw):\t"+ str(rawYddot)+"\t m/s^2 \n")
        if msg.arbitration_id == (IMU_db.get_message_by_name('Raw_Acc_Second_Half').frame_id):
            rawZddot = IMU_db.decode_message('Raw_Acc_Second_Half', msg.data)['raw_z_ddot']
            print("z_ddot (raw):\t"+ str(rawZddot)+"\t m/s^2 \n")
        continue

if __name__ == '__main__':

    main()
                 
