/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 36.7.0 Sun Feb  6 13:22:53 2022.
 */

#ifndef WHEEBBOT_IMU_CAN_H
#define WHEEBBOT_IMU_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define WHEEBBOT_IMU_CAN_IMU_HEARTBEAT_FRAME_ID (0x00u)
#define WHEEBBOT_IMU_CAN_QUAT_FIRST_HALF_FRAME_ID (0x01u)
#define WHEEBBOT_IMU_CAN_QUAT_SECOND_HALF_FRAME_ID (0x02u)
#define WHEEBBOT_IMU_CAN_EULER_FIRST_HALF_FRAME_ID (0x03u)
#define WHEEBBOT_IMU_CAN_EULER_SECOND_HALF_FRAME_ID (0x04u)
#define WHEEBBOT_IMU_CAN_GYRO_FIRST_HALF_FRAME_ID (0x05u)
#define WHEEBBOT_IMU_CAN_GYRO_SECOND_HALF_FRAME_ID (0x06u)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_FIRST_HALF_FRAME_ID (0x07u)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_SECOND_HALF_FRAME_ID (0x08u)
#define WHEEBBOT_IMU_CAN_GRAV_FIRST_HALF_FRAME_ID (0x09u)
#define WHEEBBOT_IMU_CAN_GRAV_SECOND_HALF_FRAME_ID (0x0au)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_FIRST_HALF_FRAME_ID (0x0bu)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_SECOND_HALF_FRAME_ID (0x0cu)
#define WHEEBBOT_IMU_CAN_RAW_ACC_FIRST_HALF_FRAME_ID (0x0du)
#define WHEEBBOT_IMU_CAN_RAW_ACC_SECOND_HALF_FRAME_ID (0x0eu)

/* Frame lengths in bytes. */
#define WHEEBBOT_IMU_CAN_IMU_HEARTBEAT_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_QUAT_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_QUAT_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_EULER_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_EULER_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_GYRO_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_GYRO_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_GRAV_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_GRAV_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_SECOND_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_RAW_ACC_FIRST_HALF_LENGTH (8u)
#define WHEEBBOT_IMU_CAN_RAW_ACC_SECOND_HALF_LENGTH (8u)

/* Extended or standard frame types. */
#define WHEEBBOT_IMU_CAN_IMU_HEARTBEAT_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_QUAT_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_QUAT_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_EULER_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_EULER_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_GYRO_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_GYRO_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_LINEAR_ACC_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_GRAV_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_GRAV_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_RAW_GYRO_SECOND_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_RAW_ACC_FIRST_HALF_IS_EXTENDED (0)
#define WHEEBBOT_IMU_CAN_RAW_ACC_SECOND_HALF_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


/**
 * Signals in message imu_Heartbeat.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_imu_heartbeat_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t orient_cal_status085;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t orient_cal_status055;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t imu_temp;
};

/**
 * Signals in message quat_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_quat_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float q_w;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float q_i;
};

/**
 * Signals in message quat_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_quat_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float q_j;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float q_k;
};

/**
 * Signals in message Euler_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_euler_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float euler_yaw;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float euler_pitch;
};

/**
 * Signals in message Euler_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_euler_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float euler_roll;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t is_deg;
};

/**
 * Signals in message Gyro_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_gyro_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float gyro_x;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float gyro_y;
};

/**
 * Signals in message Gyro_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_gyro_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float gyro_z;
};

/**
 * Signals in message Linear_Acc_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_linear_acc_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float x_ddot;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float y_ddot;
};

/**
 * Signals in message Linear_Acc_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_linear_acc_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float z_ddot;
};

/**
 * Signals in message Grav_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_grav_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float g_x;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float g_y;
};

/**
 * Signals in message Grav_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_grav_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float g_z;
};

/**
 * Signals in message Raw_Gyro_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_raw_gyro_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_gyro_x;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_gyro_y;
};

/**
 * Signals in message Raw_Gyro_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_raw_gyro_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_gyro_z;
};

/**
 * Signals in message Raw_Acc_First_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_raw_acc_first_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_x_ddot;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_y_ddot;
};

/**
 * Signals in message Raw_Acc_Second_Half.
 *
 * All signal values are as on the CAN bus.
 */
struct wheebbot_imu_can_raw_acc_second_half_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float raw_z_ddot;
};

/**
 * Pack message imu_Heartbeat.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_imu_heartbeat_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_imu_heartbeat_t *src_p,
    size_t size);

/**
 * Unpack message imu_Heartbeat.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_imu_heartbeat_unpack(
    struct wheebbot_imu_can_imu_heartbeat_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t wheebbot_imu_can_imu_heartbeat_orient_cal_status085_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_imu_heartbeat_orient_cal_status085_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_imu_heartbeat_orient_cal_status085_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t wheebbot_imu_can_imu_heartbeat_orient_cal_status055_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_imu_heartbeat_orient_cal_status055_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_imu_heartbeat_orient_cal_status055_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t wheebbot_imu_can_imu_heartbeat_imu_temp_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_imu_heartbeat_imu_temp_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_imu_heartbeat_imu_temp_is_in_range(uint8_t value);

/**
 * Pack message quat_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_quat_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_quat_first_half_t *src_p,
    size_t size);

/**
 * Unpack message quat_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_quat_first_half_unpack(
    struct wheebbot_imu_can_quat_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_quat_first_half_q_w_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_quat_first_half_q_w_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_quat_first_half_q_w_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_quat_first_half_q_i_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_quat_first_half_q_i_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_quat_first_half_q_i_is_in_range(float value);

/**
 * Pack message quat_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_quat_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_quat_second_half_t *src_p,
    size_t size);

/**
 * Unpack message quat_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_quat_second_half_unpack(
    struct wheebbot_imu_can_quat_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_quat_second_half_q_j_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_quat_second_half_q_j_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_quat_second_half_q_j_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_quat_second_half_q_k_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_quat_second_half_q_k_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_quat_second_half_q_k_is_in_range(float value);

/**
 * Pack message Euler_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_euler_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_euler_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Euler_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_euler_first_half_unpack(
    struct wheebbot_imu_can_euler_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_euler_first_half_euler_yaw_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_euler_first_half_euler_yaw_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_euler_first_half_euler_yaw_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_euler_first_half_euler_pitch_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_euler_first_half_euler_pitch_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_euler_first_half_euler_pitch_is_in_range(float value);

/**
 * Pack message Euler_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_euler_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_euler_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Euler_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_euler_second_half_unpack(
    struct wheebbot_imu_can_euler_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_euler_second_half_euler_roll_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_euler_second_half_euler_roll_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_euler_second_half_euler_roll_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t wheebbot_imu_can_euler_second_half_is_deg_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_euler_second_half_is_deg_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_euler_second_half_is_deg_is_in_range(uint32_t value);

/**
 * Pack message Gyro_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_gyro_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_gyro_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Gyro_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_gyro_first_half_unpack(
    struct wheebbot_imu_can_gyro_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_gyro_first_half_gyro_x_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_gyro_first_half_gyro_x_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_gyro_first_half_gyro_x_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_gyro_first_half_gyro_y_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_gyro_first_half_gyro_y_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_gyro_first_half_gyro_y_is_in_range(float value);

/**
 * Pack message Gyro_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_gyro_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_gyro_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Gyro_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_gyro_second_half_unpack(
    struct wheebbot_imu_can_gyro_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_gyro_second_half_gyro_z_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_gyro_second_half_gyro_z_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_gyro_second_half_gyro_z_is_in_range(float value);

/**
 * Pack message Linear_Acc_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_linear_acc_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_linear_acc_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Linear_Acc_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_linear_acc_first_half_unpack(
    struct wheebbot_imu_can_linear_acc_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_linear_acc_first_half_x_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_linear_acc_first_half_x_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_linear_acc_first_half_x_ddot_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_linear_acc_first_half_y_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_linear_acc_first_half_y_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_linear_acc_first_half_y_ddot_is_in_range(float value);

/**
 * Pack message Linear_Acc_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_linear_acc_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_linear_acc_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Linear_Acc_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_linear_acc_second_half_unpack(
    struct wheebbot_imu_can_linear_acc_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_linear_acc_second_half_z_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_linear_acc_second_half_z_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_linear_acc_second_half_z_ddot_is_in_range(float value);

/**
 * Pack message Grav_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_grav_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_grav_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Grav_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_grav_first_half_unpack(
    struct wheebbot_imu_can_grav_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_grav_first_half_g_x_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_grav_first_half_g_x_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_grav_first_half_g_x_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_grav_first_half_g_y_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_grav_first_half_g_y_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_grav_first_half_g_y_is_in_range(float value);

/**
 * Pack message Grav_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_grav_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_grav_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Grav_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_grav_second_half_unpack(
    struct wheebbot_imu_can_grav_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_grav_second_half_g_z_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_grav_second_half_g_z_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_grav_second_half_g_z_is_in_range(float value);

/**
 * Pack message Raw_Gyro_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_raw_gyro_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_raw_gyro_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Raw_Gyro_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_raw_gyro_first_half_unpack(
    struct wheebbot_imu_can_raw_gyro_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_gyro_first_half_raw_gyro_x_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_gyro_first_half_raw_gyro_x_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_gyro_first_half_raw_gyro_x_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_gyro_first_half_raw_gyro_y_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_gyro_first_half_raw_gyro_y_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_gyro_first_half_raw_gyro_y_is_in_range(float value);

/**
 * Pack message Raw_Gyro_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_raw_gyro_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_raw_gyro_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Raw_Gyro_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_raw_gyro_second_half_unpack(
    struct wheebbot_imu_can_raw_gyro_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_gyro_second_half_raw_gyro_z_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_gyro_second_half_raw_gyro_z_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_gyro_second_half_raw_gyro_z_is_in_range(float value);

/**
 * Pack message Raw_Acc_First_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_raw_acc_first_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_raw_acc_first_half_t *src_p,
    size_t size);

/**
 * Unpack message Raw_Acc_First_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_raw_acc_first_half_unpack(
    struct wheebbot_imu_can_raw_acc_first_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_acc_first_half_raw_x_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_acc_first_half_raw_x_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_acc_first_half_raw_x_ddot_is_in_range(float value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_acc_first_half_raw_y_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_acc_first_half_raw_y_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_acc_first_half_raw_y_ddot_is_in_range(float value);

/**
 * Pack message Raw_Acc_Second_Half.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int wheebbot_imu_can_raw_acc_second_half_pack(
    uint8_t *dst_p,
    const struct wheebbot_imu_can_raw_acc_second_half_t *src_p,
    size_t size);

/**
 * Unpack message Raw_Acc_Second_Half.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int wheebbot_imu_can_raw_acc_second_half_unpack(
    struct wheebbot_imu_can_raw_acc_second_half_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
float wheebbot_imu_can_raw_acc_second_half_raw_z_ddot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double wheebbot_imu_can_raw_acc_second_half_raw_z_ddot_decode(float value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool wheebbot_imu_can_raw_acc_second_half_raw_z_ddot_is_in_range(float value);


#ifdef __cplusplus
}
#endif

#endif
