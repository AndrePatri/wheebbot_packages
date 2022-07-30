# odrivetool installed at /usr/local/lib/python3.8/dist-packages

odrivetool dfu --> updates the odrivetool 
sudo odrivetool dfu   -->updates firmware from online rep (by usb)

odrv0.config.brake_resistance=0
odrv0.config.dc_max_negative_current=-1
odrv0.config.max_regen_current=100 

# CAN config

odrv0.axis0.config.can.node_id = 0
odrv0.axis0.config.can.encoder_rate_ms= 0
odrv0.axis0.config.can.heartbeat_rate_ms= 100
odrv0.axis0.config.can.idq_rate_ms=0

odrv0.axis1.config.can.node_id = 1
odrv0.axis1.config.can.encoder_rate_ms= 100
odrv0.axis1.config.can.heartbeat_rate_ms= 100
odrv0.axis1.config.can.idq_rate_ms=100

odrv0.can.config.baud_rate = 1000000

# AXIS0 SETTINGS and CALIBRATION

odrv0.axis0.motor.config.current_lim=10      
odrv0.axis0.controller.config.vel_limit= 5
odrv0.axis0.motor.config.pole_pairs=7
odrv0.axis0.motor.config.torque_constant=8.27/270     
odrv0.axis0.encoder.config.cpr=8192
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.calib_range = 0.05
odrv0.axis0.motor.config.calibration_current = 10.0 
odrv0.axis0.motor.config.resistance_calib_max_voltage = 2.0 

odrv0.axis0.controller.config.input_mode=1 

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.encoder.config.pre_calibrated=True
odrv0.axis0.motor.config.pre_calibrated=True
odrv0.axis0.config.startup_encoder_index_search =True

odrv0.save_configuration()

# AXIS1 SETTINGS

odrv0.axis1.motor.config.current_lim=10      
odrv0.axis1.controller.config.vel_limit=5
odrv0.axis1.motor.config.pole_pairs=7
odrv0.axis1.motor.config.torque_constant=8.27/270     
odrv0.axis1.encoder.config.cpr=8192
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.calib_range = 0.05
odrv0.axis1.motor.config.calibration_current = 10.0 
odrv0.axis1.motor.config.resistance_calib_max_voltage = 2.0 

odrv0.axis1.controller.config.input_mode=1

odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis1.encoder.config.pre_calibrated=True
odrv0.axis1.motor.config.pre_calibrated=True
odrv0.axis1.config.startup_encoder_index_search =True

odrv0.save_configuration()
odrv0.reboot()


# AXIS0 CONTROL

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  --->enters closed loop pos.control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL --->enters closed loop torque control
odrv0.axis0.requested_state = AXIS_STATE_IDLE

odrv0.axis0.controller.input_pos =          [turn]
odrv0.axis0.controller.input_vel =          [turn/s]
odrv0.axis0.controller.input_torque = 0.1   [Nm]

odrv0.axis0.encoder.pos_estimate [turns] or odrv0.axis0.encoder.pos_est_counts [counts]
odrv0.axis0.encoder.vel_estimate [turns] or odrv0.axis0.encoder.vel_est_counts [counts]
odrv0.axis0.motor.current_control.Iq_setpoint [A]
odrv0.axis0.motor.current_control.Iq_measured [A]   (Torque [N.m] = 8.27 * Current [A] / KV)

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
odrv0.axis0.encoder.shadow_count 

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])



# AXIS1 CONTROL

odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  --->enters closed loop pos.control
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL --->enters closed loop torque control
odrv0.axis1.requested_state = AXIS_STATE_IDLE

odrv0.axis1.controller.input_pos =          [turn]
odrv0.axis1.controller.input_vel =          [turn/s]
odrv0.axis1.controller.input_torque = 0.1   [Nm]

odrv0.axis1.encoder.pos_estimate [turns] or odrv0.axis0.encoder.pos_est_counts [counts]
odrv0.axis1.encoder.vel_estimate [turns] or odrv0.axis0.encoder.vel_est_counts [counts]
odrv0.axis1.motor.current_control.Iq_setpoint [A]
odrv0.axis1.motor.current_control.Iq_measured [A]   (Torque [N.m] = 8.27 * Current [A] / KV)

odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
odrv0.axis1.encoder.shadow_count 

start_liveplotter(lambda:[odrv0.axis1.encoder.pos_estimate, odrv0.axis1.controller.pos_setpoint])

# CAN config



odrv0.axis0.controller.input_pos =0.5
odrv0.axis1.controller.input_pos =0.5
rosso cel marr viol ner                                                    

# To backup a configuration:
# connect ODrive
odrivetool backup-config 
# disconnect
# connect another ODrive
odrivetool odrivetool restore-config

odrv0.axis0.config.can.encoder_rate_ms
odrv0.axis1.config.can.encoder_rate_ms






