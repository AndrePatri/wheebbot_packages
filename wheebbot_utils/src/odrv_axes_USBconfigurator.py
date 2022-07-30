#!/usr/bin/env python3

##################### CONFIGURE THE ODRIVE USING THE USB CONNECTION #########################
######## (the ODrive needs to be connected to the device running this node) ########

##################### Imports #########################

import odrive 
from odrive.enums import *

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import time

import sys

from threading import Thread

from odrive.pyfibre.fibre.libfibre import ObjectLostError

class USBConfigurator(Node):

    def __init__(self):

        super().__init__('odrv0_axes_USBconfig', start_parameter_services = True,  allow_undeclared_parameters = True, automatically_declare_parameters_from_overrides = True)

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[

        #         ('config.brake_resistance', None),
        #         ('config.dc_max_negative_current', None),
        #         ('config.max_regen_current', None),

        #         ('can.config.baud_rate', None),
                

        #         ('axis0.config.startup_encoder_index_search', None),
        #         ('axis0.config.startup_motor_calibration', None),
        #         ('axis0.config.startup_encoder_offset_calibration', None),
        #         ('axis0.config.startup_closed_loop_control', None),
        #         ('axis0.config.startup_homing', None),

        #         ('axis0.config.can.node_id', None),
        #         ('axis0.config.can.encoder_rate_ms', None),
        #         ('axis0.config.can.idq_rate_ms', None),
        #         ('axis0.config.can.heartbeat_rate_ms', None),
        #         ('axis0.config.can.idq_rate_ms', None),

        #         ('axis0.motor.config.current_lim', None),
        #         ('axis0.motor.config.pole_pairs', None),
        #         ('axis0.motor.config.torque_constant', None),
        #         ('axis0.motor.config.calibration_current', None),
        #         ('axis0.motor.config.resistance_calib_max_voltage', None),
        #         ('axis0.motor.config.pre_calibrated', None),

        #         ('axis0.encoder.config.cpr', None),
        #         ('axis0.encoder.config.mode', None),
        #         ('axis0.encoder.config.calib_range', None),
        #         ('axis0.encoder.config.use_index', None),
        #         ('axis0.encoder.config.pre_calibrated', None),

        #         ('axis0.controller.config.input_mode', None),
        #         ('axis0.controller.config.control_mode', None),
        #         ('axis0.controller.config.vel_limit', None),

                
        #         ('axis1.config.startup_encoder_index_search', None),
        #         ('axis1.config.startup_motor_calibration', None),
        #         ('axis1.config.startup_encoder_offset_calibration', None),
        #         ('axis1.config.startup_closed_loop_control', None),
        #         ('axis1.config.startup_homing', None),

        #         ('axis1.config.can.node_id', None),
        #         ('axis1.config.can.encoder_rate_ms', None),
        #         ('axis1.config.can.idq_rate_ms', None),
        #         ('axis1.config.can.heartbeat_rate_ms', None),
        #         ('axis1.config.can.idq_rate_ms', None),

        #         ('axis1.motor.config.current_lim', None),
        #         ('axis1.motor.config.pole_pairs', None),
        #         ('axis1.motor.config.torque_constant', None),
        #         ('axis1.motor.config.calibration_current', None),
        #         ('axis1.motor.config.resistance_calib_max_voltage', None),
        #         ('axis1.motor.config.pre_calibrated', None),

        #         ('axis1.encoder.config.cpr', None),
        #         ('axis1.encoder.config.mode', None),
        #         ('axis1.encoder.config.calib_range', None),
        #         ('axis1.encoder.config.use_index', None),
        #         ('axis1.encoder.config.pre_calibrated', None),

        #         ('axis1.controller.config.input_mode', None),
        #         ('axis1.controller.config.control_mode', None),
        #         ('axis1.controller.config.vel_limit', None)
        #     ])


        self.brake_res = self.get_parameter('config.brake_resistance').value
        self.dc_max_negative_current = self.get_parameter('config.dc_max_negative_current').value
        self.max_regen_current = self.get_parameter('config.max_regen_current').value

        self.can_baud_rate = self.get_parameter('can.config.baud_rate').value

        self.axis0_startup_encoder_index_search = self.get_parameter('axis0.config.startup_encoder_index_search').value
        self.axis0_is_startup_motor_calibration = self.get_parameter('axis0.config.startup_motor_calibration').value
        self.axis0_is_startup_encoder_offset_calibration = self.get_parameter('axis0.config.startup_encoder_offset_calibration').value
        self.axis0_is_startup_closed_loop_control = self.get_parameter('axis0.config.startup_closed_loop_control').value
        self.axis0_is_startup_homing = self.get_parameter('axis0.config.startup_homing').value

        self.axis0_node_id = self.get_parameter('axis0.config.can.node_id').value
        self.axis0_encoder_rate_ms = self.get_parameter('axis0.config.can.encoder_rate_ms').value
        self.axis0_heartbeat_rate_ms = self.get_parameter('axis0.config.can.heartbeat_rate_ms').value
        self.axis0_idq_rate_ms = self.get_parameter('axis0.config.can.idq_rate_ms').value

        self.axis0_current_lim = self.get_parameter('axis0.motor.config.current_lim').value
        self.axis0_pole_pairs = self.get_parameter('axis0.motor.config.pole_pairs').value
        self.axis0_torque_constant = self.get_parameter('axis0.motor.config.torque_constant').value
        self.axis0_calibration_current = self.get_parameter('axis0.motor.config.calibration_current').value
        self.axis0_resistance_calib_max_voltage = self.get_parameter('axis0.motor.config.resistance_calib_max_voltage').value
        self.axis0_motor_pre_cal = self.get_parameter('axis0.motor.config.pre_calibrated').value

        self.axis0_encoder_cpr = self.get_parameter('axis0.encoder.config.cpr').value
        self.axis0_encoder_mode = self.get_parameter('axis0.encoder.config.mode').value
        self.axis0_encoder_cal_range = self.get_parameter('axis0.encoder.config.calib_range').value
        self.axis0_encoder_pre_cal = self.get_parameter('axis0.encoder.config.pre_calibrated').value
        self.axis0_encoder_use_index = self.get_parameter('axis0.encoder.config.use_index').value

        self.axis0_input_mode = self.get_parameter('axis0.controller.config.input_mode').value
        self.axis0_control_mode = self.get_parameter('axis0.controller.config.control_mode').value
        self.axis0_vel_lim = self.get_parameter('axis0.controller.config.vel_limit').value


        self.axis1_startup_encoder_index_search = self.get_parameter('axis1.config.startup_encoder_index_search').value
        self.axis1_is_startup_motor_calibration = self.get_parameter('axis1.config.startup_motor_calibration').value
        self.axis1_is_startup_encoder_offset_calibration = self.get_parameter('axis1.config.startup_encoder_offset_calibration').value
        self.axis1_is_startup_closed_loop_control = self.get_parameter('axis1.config.startup_closed_loop_control').value
        self.axis1_is_startup_homing = self.get_parameter('axis1.config.startup_homing').value

        self.axis1_node_id = self.get_parameter('axis1.config.can.node_id').value
        self.axis1_encoder_rate_ms = self.get_parameter('axis1.config.can.encoder_rate_ms').value
        self.axis1_heartbeat_rate_ms = self.get_parameter('axis1.config.can.heartbeat_rate_ms').value
        self.axis1_idq_rate_ms = self.get_parameter('axis1.config.can.idq_rate_ms').value

        self.axis1_current_lim = self.get_parameter('axis1.motor.config.current_lim').value
        self.axis1_pole_pairs = self.get_parameter('axis1.motor.config.pole_pairs').value
        self.axis1_torque_constant = self.get_parameter('axis1.motor.config.torque_constant').value
        self.axis1_calibration_current = self.get_parameter('axis1.motor.config.calibration_current').value
        self.axis1_resistance_calib_max_voltage = self.get_parameter('axis1.motor.config.resistance_calib_max_voltage').value
        self.axis1_motor_pre_cal = self.get_parameter('axis1.motor.config.pre_calibrated').value

        self.axis1_encoder_cpr = self.get_parameter('axis1.encoder.config.cpr').value
        self.axis1_encoder_mode = self.get_parameter('axis1.encoder.config.mode').value
        self.axis1_encoder_cal_range = self.get_parameter('axis1.encoder.config.calib_range').value
        self.axis1_encoder_use_index = self.get_parameter('axis1.encoder.config.use_index').value
        self.axis1_encoder_pre_cal = self.get_parameter('axis1.encoder.config.pre_calibrated').value

        self.axis1_input_mode = self.get_parameter('axis1.controller.config.input_mode').value
        self.axis1_control_mode = self.get_parameter('axis1.controller.config.control_mode').value
        self.axis1_vel_lim = self.get_parameter('axis1.controller.config.vel_limit').value

        # Starting the USB configurator on a separate thread, since it contains a blocking method.
        self.apply_config_thread = Thread(target=self.apply_odrv_params, args=())
        self.apply_config_thread.start() 

        # Starting the USB configurator using a timer callback
        # usb_configurator.apply_odrv_params()
        # self.create_timer(1.0, self.apply_odrv_params)

    def apply_odrv_params(self):

        # Find a connected ODrive (this will block until you connect one)
        self.get_logger().info('Searching for an ODrive ...')
        my_odrive = odrive.find_any()

        self.get_logger().info("ODrive found!")
        time.sleep(1)
        
        self.get_logger().info("Bus voltage is " + str(my_odrive.vbus_voltage) + "V")
        time.sleep(1)

        self.get_logger().info("Putting both axes to IDLE to allow saving a new configuration...")
        time.sleep(1)
        
        my_odrive.axis0.requested_state = AXIS_STATE_IDLE
        while my_odrive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        my_odrive.axis1.requested_state = AXIS_STATE_IDLE
        while my_odrive.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        
        self.get_logger().info("Setting ODrive configuration based on the loaded parameters....")
        time.sleep(1)

        ## Setting up ODrive
        my_odrive.config.brake_resistance = self.brake_res
        my_odrive.config.dc_max_negative_current = self.dc_max_negative_current
        my_odrive.config.max_regen_current = self.max_regen_current

        my_odrive.can.config.baud_rate = self.can_baud_rate

        #axis0
        my_odrive.axis0.config.can.node_id = self.axis0_node_id
        my_odrive.axis0.config.can.encoder_rate_ms = self.axis0_encoder_rate_ms
        my_odrive.axis0.config.can.heartbeat_rate_ms = self.axis0_heartbeat_rate_ms
        my_odrive.axis0.config.can.idq_rate_ms = self.axis0_idq_rate_ms
        my_odrive.axis0.config.startup_encoder_index_search = self.axis0_startup_encoder_index_search
        my_odrive.axis0.config.startup_encoder_offset_calibration = self.axis0_is_startup_encoder_offset_calibration
        my_odrive.axis0.config.startup_motor_calibration = self.axis0_is_startup_motor_calibration
        my_odrive.axis0.config.startup_closed_loop_control = self.axis0_is_startup_closed_loop_control

        my_odrive.axis0.motor.config.current_lim = self.axis0_current_lim
        my_odrive.axis0.motor.config.pole_pairs = self.axis0_pole_pairs
        my_odrive.axis0.motor.config.torque_constant = self.axis0_torque_constant
        my_odrive.axis0.motor.config.calibration_current = self.axis0_calibration_current
        my_odrive.axis0.motor.config.resistance_calib_max_voltage = self.axis0_resistance_calib_max_voltage
        my_odrive.axis0.motor.config.pre_calibrated = self.axis0_motor_pre_cal 

        my_odrive.axis0.encoder.config.cpr = self.axis0_encoder_cpr
        my_odrive.axis0.encoder.config.mode = self.axis0_encoder_mode
        my_odrive.axis0.encoder.config.use_index = self.axis0_encoder_use_index
        my_odrive.axis0.encoder.config.pre_calibrated = self.axis0_encoder_pre_cal

        my_odrive.axis0.controller.config.input_mode = self.axis0_input_mode # INPUT_MODE_PASSTHROUGH
        my_odrive.axis0.controller.config.control_mode = self.axis0_control_mode # 1 torque, 2 vel, 3 position
        my_odrive.axis0.controller.config.vel_limit = self.axis0_vel_lim

        #axis1
        my_odrive.axis1.config.can.node_id = self.axis1_node_id
        my_odrive.axis1.config.can.encoder_rate_ms = self.axis1_encoder_rate_ms
        my_odrive.axis1.config.can.heartbeat_rate_ms = self.axis1_heartbeat_rate_ms
        my_odrive.axis1.config.can.idq_rate_ms = self.axis1_idq_rate_ms
        my_odrive.axis1.config.startup_encoder_index_search = self.axis1_startup_encoder_index_search
        my_odrive.axis1.config.startup_encoder_offset_calibration = self.axis1_is_startup_encoder_offset_calibration
        my_odrive.axis1.config.startup_motor_calibration = self.axis1_is_startup_motor_calibration
        my_odrive.axis1.config.startup_closed_loop_control = self.axis1_is_startup_closed_loop_control

        my_odrive.axis1.motor.config.current_lim = self.axis1_current_lim
        my_odrive.axis1.motor.config.pole_pairs = self.axis1_pole_pairs
        my_odrive.axis1.motor.config.torque_constant = self.axis1_torque_constant
        my_odrive.axis1.motor.config.calibration_current = self.axis1_calibration_current
        my_odrive.axis1.motor.config.resistance_calib_max_voltage = self.axis1_resistance_calib_max_voltage
        my_odrive.axis1.motor.config.pre_calibrated = self.axis1_motor_pre_cal 

        my_odrive.axis1.encoder.config.cpr = self.axis1_encoder_cpr
        my_odrive.axis1.encoder.config.mode = self.axis1_encoder_mode
        my_odrive.axis1.encoder.config.use_index = self.axis1_encoder_use_index
        my_odrive.axis1.encoder.config.pre_calibrated = self.axis1_encoder_pre_cal

        my_odrive.axis1.controller.config.input_mode = self.axis1_input_mode # INPUT_MODE_PASSTHROUGH
        my_odrive.axis1.controller.config.control_mode = self.axis1_control_mode # 1 torque, 2 vel, 3 position
        my_odrive.axis1.controller.config.vel_limit = self.axis1_vel_lim
   
        try:
            self.get_logger().info("Configuration was read successfully. Saving configuration on the ODrive...")
            time.sleep(1)
            my_odrive.save_configuration()
            
        finally:
            self.get_logger().info("Configuration set. ODrive will reboot. Please ignore the fibre.libfibre.ObjectLostError.")
            time.sleep(2)
            pass
        

def main(args=None):
    rclpy.init(args = args)
    try:
        usb_configurator = USBConfigurator()    
        rclpy.spin(usb_configurator)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        usb_configurator.destroy_node()
    
if __name__ == '__main__':

    main()