#!/usr/bin/env python3

import cantools

import os

def main():

    ######## GETTING DIRECTORIES ########

    PACKAGE_DIR = os.path.normpath(os.getcwd() + os.sep + os.pardir)
    OUTPUT_DBC_PATH = os.path.join(PACKAGE_DIR, 'config/CAN_config', 'imu_can.dbc')

    ######## DBC DATABASE FOR HANDLING IMU DATA OVER CAN ########

    # class cantools.database.can.Signal(name, start, length, byte_order='little_endian', is_signed=False, initial=None, scale=1, offset=0, 
    # minimum=None, maximum=None, unit=None, choices=None, dbc_specifics=None, comment=None, receivers=None, is_multiplexer=False, multiplexer_ids=None, 
    # multiplexer_signal=None, is_float=False, decimal=None)[source]

    ## 0x000 - Heartbeat message (IMU status&C) Message --> highest priority message ##

    orientCalStatus085 = cantools.database.can.Signal("orient_cal_status085", 0, 8, unit="adimensional") 
    gyroCalStatus055 = cantools.database.can.Signal("orient_cal_status055", 8, 8, unit="adimensional") 
    imuTemp=cantools.database.can.Signal("imu_temp", 16, 8, is_float=False, unit="Celsius") 

    imuHeartbeat = cantools.database.can.Message(
        0x000, "imu_Heartbeat", 8, [orientCalStatus085,gyroCalStatus055,imuTemp], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Orientation data ( QUATERNION ) : Q(t)=q_w(t) + q_i(t)*i + q_j(t)*j + q_k(t)*k ###
    ### Each component is 32 bit --> the orientation data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x001 - first half: q_w and q_i ##

    qwSignal = cantools.database.can.Signal("q_w", 0, 32, is_float=True, unit="normalized")
    qiSignal = cantools.database.can.Signal("q_i", 32, 32, is_float=True, unit="normalized")

    quatFirstHalf = cantools.database.can.Message(
        0x001, "quat_First_Half", 8, [qwSignal, qiSignal], bus_name="imu_Can_Bus")

    ## 0x002 - second half: q_j and q_k ##

    qjSignal = cantools.database.can.Signal("q_j", 0, 32, is_float=True,unit="normalized")
    qkSignal = cantools.database.can.Signal("q_k", 32, 32, is_float=True,unit="normalized")

    quatSecondHalf = cantools.database.can.Message(
        0x002, "quat_Second_Half", 8, [qjSignal, qkSignal], bus_name="imu_Can_Bus")


    ### Orientation data ( EULER ): Yaw-Pitch-Roll ###
    ### Each component is 32 bit --> the orientation data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x003 - first half: Yaw and Pitch ##

    YawSignal = cantools.database.can.Signal("Euler_Yaw", 0, 32, is_float=True,unit="rad")
    PitchSignal = cantools.database.can.Signal("Euler_Pitch", 32, 32, is_float=True,unit="rad")

    EulerFirstHalf = cantools.database.can.Message(
        0x003, "Euler_First_Half", 8, [YawSignal, PitchSignal], bus_name="imu_Can_Bus")

    ## 0x004 - second half: Roll (32 bits left unused) ##

    RollSignal = cantools.database.can.Signal("Euler_Roll", 0, 32, is_float=True, unit="rad")
    IsDeg = cantools.database.can.Signal("Is_Deg", 32, 32, unit="adimensional")

    EulerSecondHalf = cantools.database.can.Message(
        0x004, "Euler_Second_Half", 8, [RollSignal,IsDeg], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Gyroscope data : X-Y-Z ###
    ### Each component is 32 bit --> the data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x005 - first half: angular vel on X and Y ##

    gyroX = cantools.database.can.Signal("gyro_x", 0, 32, is_float=True,unit="rad/s")
    gyroY = cantools.database.can.Signal("gyro_y", 32, 32, is_float=True,unit="rad/s")

    GyroFirstHalf = cantools.database.can.Message(
        0x005, "Gyro_First_Half", 8, [gyroX, gyroY], bus_name="imu_Can_Bus")

    ## 0x006 - second half: angular vel on Z ##

    gyroZ = cantools.database.can.Signal("gyro_z", 0, 32, is_float=True,unit="rad/s")

    GyroSecondHalf = cantools.database.can.Message(
        0x006, "Gyro_Second_Half", 8, [gyroZ], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Linear acceleration data : X-Y-Z ###
    ### Each component is 32 bit --> the data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x007 - first half: linear acc on X and Y ##

    linAccX = cantools.database.can.Signal("x_ddot", 0, 32, is_float=True,unit="m/s^2")
    linAccY = cantools.database.can.Signal("y_ddot", 32, 32, is_float=True,unit="m/s^2")

    linAccFirstHalf = cantools.database.can.Message(
        0x007, "Linear_Acc_First_Half", 8, [linAccX, linAccY], bus_name="imu_Can_Bus")

    ## 0x008 - second half: linear acc on Z ##

    linAccZ = cantools.database.can.Signal("z_ddot", 0, 32, is_float=True,unit="m/s^2")

    linAccSecondHalf = cantools.database.can.Message(
        0x008, "Linear_Acc_Second_Half", 8, [linAccZ], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Gravity data : X-Y-Z ###
    ### Each component is 32 bit --> the data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x00A - first half: gravity on X and Y ##

    gravX = cantools.database.can.Signal("g_x", 0, 32, is_float=True,unit="m/s^2")
    gravY = cantools.database.can.Signal("g_y", 32, 32, is_float=True,unit="m/s^2")

    gravFirstHalf = cantools.database.can.Message(
        0x009, "Grav_First_Half", 8, [gravX, gravY], bus_name="imu_Can_Bus")

    ## 0x010 - second half: gravity on Z ##

    gravZ = cantools.database.can.Signal("g_z", 0, 32, is_float=True,unit="m/s^2")

    gravSecondHalf = cantools.database.can.Message(
        0x00A, "Grav_Second_Half", 8, [gravZ], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Raw Gyroscope data : X-Y-Z ###
    ### Each component is 32 bit --> the data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x00B - first half: angular vel on X and Y ##

    rawGyroX = cantools.database.can.Signal("raw_gyro_x", 0, 32, is_float=True,unit="rad/s")
    rawGyroY = cantools.database.can.Signal("raw_gyro_y", 32, 32, is_float=True,unit="rad/s")

    rawGyroFirstHalf = cantools.database.can.Message(
        0x00B, "Raw_Gyro_First_Half", 8, [rawGyroX, rawGyroY], bus_name="imu_Can_Bus")

    ## 0x00C - second half: angular vel on Z ##

    rawGyroZ = cantools.database.can.Signal("raw_gyro_z", 0, 32, is_float=True,unit="rad/s")

    rawGyroSecondHalf = cantools.database.can.Message(
        0x00C, "Raw_Gyro_Second_Half", 8, [rawGyroZ], bus_name="imu_Can_Bus")

    ########################################################################################################

    ### Raw acceleration data : X-Y-Z ###
    ### Each component is 32 bit --> the data needs to be split into two messages (remember max data length is 64 bits and a single float occupies 32 bits) ###

    ## 0x00D - first half: linear acc on X and Y ##

    rawAccX = cantools.database.can.Signal("raw_x_ddot", 0, 32, is_float=True,unit="m/s^2")
    rawAccY = cantools.database.can.Signal("raw_y_ddot", 32, 32, is_float=True,unit="m/s^2")

    rawAccFirstHalf = cantools.database.can.Message(
        0x00D, "Raw_Acc_First_Half", 8, [rawAccX, rawAccY], bus_name="imu_Can_Bus")

    ## 0x00E - second half: linear acc on Z ##

    rawAccZ = cantools.database.can.Signal("raw_z_ddot", 0, 32, is_float=True,unit="m/s^2")

    rawAccSecondHalf = cantools.database.can.Message(
        0x00E, "Raw_Acc_Second_Half", 8, [rawAccZ], bus_name="imu_Can_Bus")

    db = cantools.database.can.Database(
        [
            imuHeartbeat,
            quatFirstHalf,
            quatSecondHalf,
            EulerFirstHalf,
            EulerSecondHalf,
            GyroFirstHalf,
            GyroSecondHalf,
            linAccFirstHalf,
            linAccSecondHalf,
            gravFirstHalf,
            gravSecondHalf,
            rawGyroFirstHalf,
            rawGyroSecondHalf,
            rawAccFirstHalf,
            rawAccSecondHalf
        ]
    )

    cantools.database.dump_file(db, OUTPUT_DBC_PATH)
    db = cantools.database.load_file(OUTPUT_DBC_PATH)

    print(db)

if __name__ == '__main__':

    main()
