/////////////////////////////////////////////// USEFUL DEFINITIONS ///////////////////////////////////////////////////

#define BNO08X_RESET -1 //BNO085 reset pin; I2C does not need it, so -1 is used

/////////////////////////////////////////////// IMPORTED LIBRARIES ///////////////////////////////////////////////////

// Including all Arduino-related stuff
#include <Arduino.h>

// Using BNO08x IMU library:
#include <Adafruit_BNO08x.h>

// Using (modified) Seedstudio CAN library:
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>

// Using BNO055 library (if applicable)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Importing the libraries used for packing/unpacking and coding/decoding IMU messages on the CAN bus (almost completely generated useing "cantools" Python utility)

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

///////////////////////////////////////////////  USED STRUCTURES  ////////////////////////////////////////////////////

// Structure to hold the report settings; to enable/disable one or more reports, set the values to true/false (ATTENTION: enable only one report)
struct enabled_reports {

  const bool IS_SH2_GYRO_INTEGRATED_RV = false; // Top frequency is reported to be 1000Hz (but freq is somewhat variable); potentially lessa accurate, but provides also a calibrated gyroscope reading
  const bool IS_SH2_ARVR_STABILIZED_RV = false; // top frequency is about 250Hz but this report is more accurate
  const bool IS_SH2_ROTATION_VECTOR = false; // Most accurate (uses gyro,accel and magn)
  const bool IS_SH2_GEOMAGNETIC_ROTATION_VECTOR = false; // no gyroscope (less power)
  const bool IS_SH2_GAME_ROTATION_VECTOR = true; // no magnetometer (no yaw jumps)

  const bool IS_SH2_ACCELEROMETER = false; // total acceleration
  const bool IS_SH2_GYROSCOPE_CALIBRATED = false;
  const bool IS_SH2_MAGNETIC_FIELD_CALIBRATED = false;
  const bool IS_SH2_LINEAR_ACCELERATION = false; // total acceleration - gravity
  const bool IS_SH2_GRAVITY = false;
  
  const bool IS_SH2_RAW_ACCELEROMETER = false;
  const bool IS_SH2_RAW_GYROSCOPE = false;
  const bool IS_SH2_RAW_MAGNETOMETER = false;

} enbld_reports;

struct message_IDs {
  unsigned long imu_status_ID = 0x000;
  unsigned long quat_first_half_ID = 0x001;
  unsigned long quat_second_half_ID = 0x002;
  unsigned long eul_first_half_ID = 0x003;
  unsigned long eul_second_half_ID = 0x004;
  unsigned long gyro_first_half_ID = 0x005;
  unsigned long gyro_second_half_ID = 0x006;
  unsigned long lin_acc_first_half_ID = 0x007;
  unsigned long lin_acc_second_half_ID = 0x008;
  unsigned long grav_first_half_ID = 0x009;
  unsigned long grav_second_half_ID = 0x00A;

} messageIDs;


////// IMU CAN message handling //////

struct imu_status_t {

  uint32_t imu_cal_status;

} src_imu_heartbeat = {0};

struct imu_quat_first_half_t {

  float q_w;
  float q_i;

} src_p_quat_first_half = {0.0, 0.0};

struct imu_quat_second_half_t {

  float q_j;
  float q_k;

} src_p_quat_second_half = {0.0, 0.0};

struct imu_eul_first_half_t {

  float yaw;
  float pitch;

} src_p_eul_first_half = {0.0, 0.0};

struct imu_eul_second_half_t {

  float roll;
  uint32_t is_deg;

} src_p_eul_second_half = {0.0, 0};

struct imu_gyro_first_half_t {

  float gyro_x;
  float gyro_y;

} src_p_gyro_first_half = {0.0, 0.0};

struct imu_gyro_second_half_t {

  float gyro_z;
  uint32_t imu_cal_status;

} src_p_gyro_second_half = {0.0, 0};

struct imu_lin_acc_first_half_t {

  float x_ddot;
  float y_ddot;

} src_p_lin_acc_first_half = {0.0, 0.0};

struct imu_lin_acc_second_half_t {

  float z_ddot;

} src_p_lin_acc_second_half = {0.0};

static struct imu_grav_first_half_t {

  float g_x;
  float g_y;

} src_p_grav_first_half = {0.0, 0.0};

struct imu_grav_second_half_t {

  float g_z;

} src_p_grav_second_half = {0.0};

///////////////////////////////////////////////  INITIALIZATIONS  ////////////////////////////////////////////////////


////// BNO085 //////

// IMU object initialization
Adafruit_BNO08x  bno08x(BNO08X_RESET);

// Structure which holds reports, IMU name, etc...
sh2_SensorValue_t sensorValue;

// Report rate (the BNO085 will: 0.9 * RequestedRate <= ConfiguredRate <= 2.1 * RequestedRate)

long reportIntervalUs = 5000; // report rate; 2000Us->500Hz; 4000Us->250Hz; 5000Us->200Hz; 10000Us->100Hz; 20000Us->50Hz;40000Us->25Hz

// Report orientation in quaternions or euler angles?
bool is_quat = false; // whether of not to use quaternions or euler engles
uint32_t is_deg = 1; // if using euler angles, whether or not to use degrees
float G=9.80665; //gravity acceleration

////// BNO055, if applicable //////

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

////// CAN-SPI //////

const int SPI_CS_PIN = 10; // chip select pin
const int CAN_INT_PIN = 8; // interrupt pin
mcp2515_can CAN(SPI_CS_PIN);

////// IMU over CAN message encoding and packing utilities //////

uint8_t dst_p_imu_heartbeat[8];// 8 byte data destination variable for CAN msgs
uint8_t dst_p_quat_first_half[8];
uint8_t dst_p_quat_second_half[8];
uint8_t dst_p_eul_first_half[8];
uint8_t dst_p_eul_second_half[8];
uint8_t dst_p_gyro_first_half[8];
uint8_t dst_p_gyro_second_half[8];
uint8_t dst_p_lin_acc_first_half[8];
uint8_t dst_p_lin_acc_second_half[8];
uint8_t dst_p_grav_first_half[8];
uint8_t dst_p_grav_second_half[8];

/////////////////////////////////////////// FUNCTION DEFINITIONS /////////////////////////////////////////////////////

// Function to set report types
void setReports(struct enabled_reports enbld_rep, long report_interval) {

  Serial.println("Setting desired reports");

  if (enbld_rep.IS_SH2_GYRO_INTEGRATED_RV == true) {
    if (! bno08x.enableReport(SH2_GYRO_INTEGRATED_RV,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_ARVR_STABILIZED_RV == true) {
    if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GEOMAGNETIC_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GAME_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_ACCELEROMETER == true) {
    if (! bno08x.enableReport(SH2_ACCELEROMETER,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GYROSCOPE_CALIBRATED == true) {
    if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_MAGNETIC_FIELD_CALIBRATED == true) {
    if (! bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_LINEAR_ACCELERATION == true) {
    if (! bno08x.enableReport(SH2_LINEAR_ACCELERATION,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GRAVITY == true) {
    if (! bno08x.enableReport(SH2_GRAVITY,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GEOMAGNETIC_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_GAME_ROTATION_VECTOR == true) {
    if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_RAW_ACCELEROMETER == true) {
    if (! bno08x.enableReport(SH2_RAW_ACCELEROMETER,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_RAW_GYROSCOPE == true) {
    if (! bno08x.enableReport(SH2_RAW_GYROSCOPE),report_interval) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  if (enbld_rep.IS_SH2_RAW_MAGNETOMETER == true) {
    if (! bno08x.enableReport(SH2_RAW_MAGNETOMETER,report_interval)) { // Setting report type/types and report rate
      Serial.println("Could not enable stabilized remote vector");
    }
  }
}

// Function to convert quaternions reading to euler angles (NOTE: might introduce a computational overhead on the microprocessor)
void quaternionToEuler(float q_w, float q_i, float q_j, float q_k, struct imu_eul_first_half_t* imu_eul_first_half, struct imu_eul_second_half_t* imu_eul_second_half, uint32_t is_deg = true) {

  float sqr = sq(q_w);
  float sqi = sq(q_i);
  float sqj = sq(q_j);
  float sqk = sq(q_k);

  imu_eul_first_half->yaw = atan2(2.0 * (q_i * q_j + q_k * q_w), (sqi - sqj - sqk + sqr));
  imu_eul_first_half->pitch = asin(-2.0 * (q_i * q_k - q_j * q_w) / (sqi + sqj + sqk + sqr));
  imu_eul_second_half->roll = atan2(2.0 * (q_j * q_k + q_i * q_w), (-sqi - sqj + sqk + sqr));
  imu_eul_second_half->is_deg = is_deg;

  if (is_deg == 1) {
    imu_eul_first_half->yaw *= RAD_TO_DEG;
    imu_eul_first_half->pitch *= RAD_TO_DEG;
    imu_eul_second_half->roll *= RAD_TO_DEG;
  }
}

// Functions for handling CAN msg packing

uint8_t pack_left_shift_u32(
  uint32_t value,
  uint8_t shift,
  uint8_t mask)
{
  return (uint8_t)((uint8_t)(value << shift) & mask);
}

uint8_t pack_right_shift_u32(
  uint32_t value,
  uint8_t shift,
  uint8_t mask)
{
  return (uint8_t)((uint8_t)(value >> shift) & mask);
}

int imuHeartbeatPack(
  uint8_t *dst_p,
  const struct imu_status_t *src_p)
{
  uint32_t cal_st;
 
  Serial.print("\n Calibration status (0-3):\t"); Serial.print(src_p->imu_cal_status, 8);

  memset(dst_p, 0, 8);

  memcpy(&cal_st, &(src_p->imu_cal_status), sizeof(cal_st));
  dst_p[0] |= pack_left_shift_u32(cal_st, 0u, 0xffu);
  dst_p[1] |= pack_right_shift_u32(cal_st, 8u, 0xffu);
  dst_p[2] |= pack_right_shift_u32(cal_st, 16u, 0xffu);
  dst_p[3] |= pack_right_shift_u32(cal_st, 24u, 0xffu);

  return (8);
}

int orientQuatPack(
  uint8_t *dst_p_first,
  uint8_t *dst_p_second,
  const struct imu_quat_first_half_t *src_p_first,
  const struct imu_quat_second_half_t *src_p_second)
{
  uint32_t q_i;
  uint32_t q_w;
  uint32_t q_j;
  uint32_t q_k;

  Serial.print("\n source q_w:\t"); Serial.print(src_p_first->q_w, 8); Serial.print("\t"); Serial.print("source q_i:\t"); Serial.print(src_p_first->q_i, 8); Serial.print("\n"); // checking if source package is read correctly
  Serial.print("source q_j:\t"); Serial.print(src_p_second->q_j, 8); Serial.print("\t"); Serial.print("source q_k:\t"); Serial.print(src_p_second->q_k, 8); Serial.print("\n"); // checking if source package is read correctly

  memset(dst_p_first, 0, 8);

  memcpy(&q_w, &(src_p_first->q_w), sizeof(q_w));
  dst_p_first[0] |= pack_left_shift_u32(q_w, 0u, 0xffu);
  dst_p_first[1] |= pack_right_shift_u32(q_w, 8u, 0xffu);
  dst_p_first[2] |= pack_right_shift_u32(q_w, 16u, 0xffu);
  dst_p_first[3] |= pack_right_shift_u32(q_w, 24u, 0xffu);

  memcpy(&q_i, &src_p_first->q_i, sizeof(q_i));
  dst_p_first[4] |= pack_left_shift_u32(q_i, 0u, 0xffu);
  dst_p_first[5] |= pack_right_shift_u32(q_i, 8u, 0xffu);
  dst_p_first[6] |= pack_right_shift_u32(q_i, 16u, 0xffu);
  dst_p_first[7] |= pack_right_shift_u32(q_i, 24u, 0xffu);

  memset(dst_p_second, 0, 8);

  memcpy(&q_j, &src_p_second->q_j, sizeof(q_j));
  dst_p_second[0] |= pack_left_shift_u32(q_j, 0u, 0xffu);
  dst_p_second[1] |= pack_right_shift_u32(q_j, 8u, 0xffu);
  dst_p_second[2] |= pack_right_shift_u32(q_j, 16u, 0xffu);
  dst_p_second[3] |= pack_right_shift_u32(q_j, 24u, 0xffu);

  memcpy(&q_k, &src_p_second->q_k, sizeof(q_k));
  dst_p_second[4] |= pack_left_shift_u32(q_k, 0u, 0xffu);
  dst_p_second[5] |= pack_right_shift_u32(q_k, 8u, 0xffu);
  dst_p_second[6] |= pack_right_shift_u32(q_k, 16u, 0xffu);
  dst_p_second[7] |= pack_right_shift_u32(q_k, 24u, 0xffu);

  return (8);
}

int orientEulPack(
  uint8_t *dst_p_first,
  uint8_t *dst_p_second,
  const struct imu_eul_first_half_t *src_p_first,
  const struct imu_eul_second_half_t *src_p_second)
{

  uint32_t yaw;
  uint32_t pitch;
  uint32_t roll;
  uint32_t is_deg;

  Serial.print("\n source yaw:\t"); Serial.print(src_p_first->yaw, 8); Serial.print("\t"); Serial.print("source pitch:\t"); Serial.print(src_p_first->pitch, 8); Serial.print("\t"); // checking if source package is read correctly
  Serial.print("source roll:\t"); Serial.print(src_p_second->roll, 8); Serial.print("\t"); Serial.print(" Is deg? :\t"); Serial.print(src_p_second->is_deg); Serial.print("\t");

  memset(dst_p_first, 0, 8);

  memcpy(&yaw, &(src_p_first->yaw), sizeof(yaw));
  dst_p_first[0] |= pack_left_shift_u32(yaw, 0u, 0xffu);
  dst_p_first[1] |= pack_right_shift_u32(yaw, 8u, 0xffu);
  dst_p_first[2] |= pack_right_shift_u32(yaw, 16u, 0xffu);
  dst_p_first[3] |= pack_right_shift_u32(yaw, 24u, 0xffu);

  memcpy(&pitch, &src_p_first->pitch, sizeof(pitch));
  dst_p_first[4] |= pack_left_shift_u32(pitch, 0u, 0xffu);
  dst_p_first[5] |= pack_right_shift_u32(pitch, 8u, 0xffu);
  dst_p_first[6] |= pack_right_shift_u32(pitch, 16u, 0xffu);
  dst_p_first[7] |= pack_right_shift_u32(pitch, 24u, 0xffu);

  memset(dst_p_second, 0, 8);

  memcpy(&roll, &src_p_second->roll, sizeof(roll));
  dst_p_second[0] |= pack_left_shift_u32(roll, 0u, 0xffu);
  dst_p_second[1] |= pack_right_shift_u32(roll, 8u, 0xffu);
  dst_p_second[2] |= pack_right_shift_u32(roll, 16u, 0xffu);
  dst_p_second[3] |= pack_right_shift_u32(roll, 24u, 0xffu);

  memcpy(&is_deg, &src_p_second->is_deg, sizeof(is_deg));
  dst_p_second[4] |= pack_left_shift_u32(is_deg, 0u, 0xffu);
  dst_p_second[5] |= pack_right_shift_u32(is_deg, 8u, 0xffu);
  dst_p_second[6] |= pack_right_shift_u32(is_deg, 16u, 0xffu);
  dst_p_second[7] |= pack_right_shift_u32(is_deg, 24u, 0xffu);

  return (8);
}

int gyroPack(
  uint8_t *dst_p_first,
  uint8_t *dst_p_second,
  const struct imu_gyro_first_half_t *src_p_first,
  const struct imu_gyro_second_half_t *src_p_second)
{
  uint32_t omega_x;
  uint32_t omega_y;
  uint32_t omega_z;
  uint32_t cal_status;

  Serial.print("\n Omega x:\t"); Serial.print(src_p_first->gyro_x, 8); Serial.print("\t"); Serial.print("Omega_y:\t"); Serial.print(src_p_first->gyro_y, 8); Serial.print("\t"); // checking if source package is read correctly
  Serial.print("Omega_z:\t"); Serial.print(src_p_second->gyro_z, 8); Serial.print("\n"); Serial.print("Cal status:\t"); Serial.print(src_p_second->imu_cal_status, 8); Serial.print("\n");

  memset(dst_p_first, 0, 8);

  memcpy(&omega_x, &src_p_first->gyro_x, sizeof(omega_x));
  dst_p_first[0] |= pack_left_shift_u32(omega_x, 0u, 0xffu);
  dst_p_first[1] |= pack_right_shift_u32(omega_x, 8u, 0xffu);
  dst_p_first[2] |= pack_right_shift_u32(omega_x, 16u, 0xffu);
  dst_p_first[3] |= pack_right_shift_u32(omega_x, 24u, 0xffu);
  memcpy(&omega_y, &src_p_first->gyro_y, sizeof(omega_y));
  dst_p_first[4] |= pack_left_shift_u32(omega_y, 0u, 0xffu);
  dst_p_first[5] |= pack_right_shift_u32(omega_y, 8u, 0xffu);
  dst_p_first[6] |= pack_right_shift_u32(omega_y, 16u, 0xffu);
  dst_p_first[7] |= pack_right_shift_u32(omega_y, 24u, 0xffu);
  memcpy(&omega_z, &src_p_second->gyro_z, sizeof(omega_z));

  memset(dst_p_second, 0, 8);

  dst_p_second[0] |= pack_left_shift_u32(omega_z, 0u, 0xffu);
  dst_p_second[1] |= pack_right_shift_u32(omega_z, 8u, 0xffu);
  dst_p_second[2] |= pack_right_shift_u32(omega_z, 16u, 0xffu);
  dst_p_second[3] |= pack_right_shift_u32(omega_z, 24u, 0xffu);
  memcpy(&cal_status, &src_p_second->imu_cal_status, sizeof(cal_status));
  dst_p_second[4] |= pack_left_shift_u32(cal_status, 0u, 0xffu);
  dst_p_second[5] |= pack_right_shift_u32(cal_status, 8u, 0xffu);
  dst_p_second[6] |= pack_right_shift_u32(cal_status, 16u, 0xffu);
  dst_p_second[7] |= pack_right_shift_u32(cal_status, 24u, 0xffu);

  return (8);
}

int linAccPack(
  uint8_t *dst_p_first,
  uint8_t *dst_p_second,
  const struct imu_lin_acc_first_half_t *src_p_first,
  const struct imu_lin_acc_second_half_t *src_p_second)
{
  uint32_t x_ddot;
  uint32_t y_ddot;
  uint32_t z_ddot;

  Serial.print("\n x_ddot:\t"); Serial.print(src_p_first->x_ddot, 8); Serial.print("\t"); Serial.print("y_ddot:\t"); Serial.print(src_p_first->y_ddot, 8); Serial.print("\t"); // checking if source package is read correctly
  Serial.print("z_ddot:\t"); Serial.print(src_p_second->z_ddot, 8); Serial.print("\n");

  memset(dst_p_first, 0, 8);

  memcpy(&x_ddot, &src_p_first->x_ddot, sizeof(x_ddot));
  dst_p_first[0] |= pack_left_shift_u32(x_ddot, 0u, 0xffu);
  dst_p_first[1] |= pack_right_shift_u32(x_ddot, 8u, 0xffu);
  dst_p_first[2] |= pack_right_shift_u32(x_ddot, 16u, 0xffu);
  dst_p_first[3] |= pack_right_shift_u32(x_ddot, 24u, 0xffu);
  memcpy(&y_ddot, &src_p_first->y_ddot, sizeof(y_ddot));
  dst_p_first[4] |= pack_left_shift_u32(y_ddot, 0u, 0xffu);
  dst_p_first[5] |= pack_right_shift_u32(y_ddot, 8u, 0xffu);
  dst_p_first[6] |= pack_right_shift_u32(y_ddot, 16u, 0xffu);
  dst_p_first[7] |= pack_right_shift_u32(y_ddot, 24u, 0xffu);

  memset(dst_p_second, 0, 8);

  memcpy(&z_ddot, &src_p_second->z_ddot, sizeof(z_ddot));
  dst_p_second[0] |= pack_left_shift_u32(z_ddot, 0u, 0xffu);
  dst_p_second[1] |= pack_right_shift_u32(z_ddot, 8u, 0xffu);
  dst_p_second[2] |= pack_right_shift_u32(z_ddot, 16u, 0xffu);
  dst_p_second[3] |= pack_right_shift_u32(z_ddot, 24u, 0xffu);

  return (8);
}

int gravPack(
  uint8_t *dst_p_first,
  uint8_t *dst_p_second,
  const struct imu_grav_first_half_t *src_p_first,
  const struct imu_grav_second_half_t *src_p_second)
{
  uint32_t g_x;
  uint32_t g_y;
  uint32_t g_z;

  Serial.print("\n g_x:\t"); Serial.print(src_p_first->g_x, 8); Serial.print("\t"); Serial.print("g_y:\t"); Serial.print(src_p_first->g_y, 8); Serial.print("\t"); // checking if source package is read correctly
  Serial.print("g_z:\t"); Serial.print(src_p_second->g_z, 8); Serial.print("\n");

  memset(dst_p_first, 0, 8);

  memcpy(&g_x, &src_p_first->g_x, sizeof(g_x));
  dst_p_first[0] |= pack_left_shift_u32(g_x, 0u, 0xffu);
  dst_p_first[1] |= pack_right_shift_u32(g_x, 8u, 0xffu);
  dst_p_first[2] |= pack_right_shift_u32(g_x, 16u, 0xffu);
  dst_p_first[3] |= pack_right_shift_u32(g_x, 24u, 0xffu);
  memcpy(&g_y, &src_p_first->g_y, sizeof(g_y));
  dst_p_first[4] |= pack_left_shift_u32(g_y, 0u, 0xffu);
  dst_p_first[5] |= pack_right_shift_u32(g_y, 8u, 0xffu);
  dst_p_first[6] |= pack_right_shift_u32(g_y, 16u, 0xffu);
  dst_p_first[7] |= pack_right_shift_u32(g_y, 24u, 0xffu);

  memset(dst_p_second, 0, 8);

  memcpy(&g_z, &src_p_second->g_z, sizeof(g_z));
  dst_p_second[0] |= pack_left_shift_u32(g_z, 0u, 0xffu);
  dst_p_second[1] |= pack_right_shift_u32(g_z, 8u, 0xffu);
  dst_p_second[2] |= pack_right_shift_u32(g_z, 16u, 0xffu);
  dst_p_second[3] |= pack_right_shift_u32(g_z, 24u, 0xffu);

  return (8);
}


// send IMU heartbeat over CAN
void sendImuHeartbeatCAN(uint8_t *dst_p_imu_heartbeat,
                            struct imu_status_t *src_p_imu_status,
                            struct message_IDs *msgIDs) {


    int check = imuHeartbeatPack(dst_p_imu_heartbeat, src_p_imu_status);

    CAN.sendMsgBuf(msgIDs->imu_status_ID, 0, 8, dst_p_imu_heartbeat);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)

}

// send orientation quaternion over CAN
void sendOrientationOverCAN(uint8_t *dst_p_quat_first_half, uint8_t *dst_p_quat_second_half,
                            uint8_t *dst_p_eul_first_half, uint8_t *dst_p_eul_second_half,
                            struct imu_quat_first_half_t *src_p_first_quat, struct imu_quat_second_half_t *src_p_second_quat,
                            struct imu_eul_first_half_t *src_p_first_eul, struct imu_eul_second_half_t *src_p_second_eul,
                            struct message_IDs *msgIDs,
                            bool is_quaternion) {

  if (is_quaternion == 1) {

    int check = orientQuatPack(dst_p_quat_first_half, dst_p_quat_second_half, src_p_first_quat, src_p_second_quat);

    CAN.sendMsgBuf(msgIDs->quat_first_half_ID, 0, 8, dst_p_quat_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
    CAN.sendMsgBuf(msgIDs->quat_second_half_ID, 0, 8, dst_p_quat_second_half);

  }


  else {

    int check = orientEulPack(dst_p_eul_first_half, dst_p_eul_second_half, src_p_first_eul, src_p_second_eul);

    CAN.sendMsgBuf(msgIDs->eul_first_half_ID, 0, 8, dst_p_eul_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
    CAN.sendMsgBuf(msgIDs->eul_second_half_ID, 0, 8, dst_p_eul_second_half);

  }

}

//// send gyro readings data over CAN
void sendGyroOverCAN(uint8_t *dst_p_gyro_first_half, uint8_t *dst_p_gyro_second_half,
                     struct imu_gyro_first_half_t *src_p_first_gyro, struct imu_gyro_second_half_t *src_p_second_gyro,
                     struct message_IDs *msgIDs) {


  //       src_p_first_gyro->gyro_x=55.0;
  //       src_p_first_gyro->gyro_y=54.0;
  //       src_p_second_gyro->gyro_z=53.0;

  int check = gyroPack(dst_p_gyro_first_half, dst_p_gyro_second_half, src_p_first_gyro, src_p_second_gyro);

  CAN.sendMsgBuf(msgIDs->gyro_first_half_ID, 0, 8, dst_p_gyro_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
  CAN.sendMsgBuf(msgIDs->gyro_second_half_ID, 0, 8, dst_p_gyro_second_half);

}

void sendLinAccOverCAN(uint8_t *dst_p_lin_acc_first_half, uint8_t *dst_p_lin_acc_second_half,
                       struct imu_lin_acc_first_half_t *src_p_first_lin_acc, struct imu_lin_acc_second_half_t *src_p_second_lin_acc,
                       struct message_IDs *msgIDs) {

//  src_p_first_lin_acc->x_ddot = 10.0;
//  src_p_first_lin_acc->y_ddot = 11.0;
//  src_p_second_lin_acc->z_ddot = 12.0;

  int check = linAccPack(dst_p_lin_acc_first_half, dst_p_lin_acc_second_half, src_p_first_lin_acc, src_p_second_lin_acc);

  CAN.sendMsgBuf(msgIDs->lin_acc_first_half_ID, 0, 8, dst_p_lin_acc_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
  CAN.sendMsgBuf(msgIDs->lin_acc_second_half_ID, 0, 8, dst_p_lin_acc_second_half);

}

void sendGravOverCAN(uint8_t *dst_p_grav_first_half, uint8_t *dst_p_grav_second_half,
                     struct imu_grav_first_half_t *src_p_first_grav, struct imu_grav_second_half_t *src_p_second_grav,
                     struct message_IDs *msgIDs) {

//  src_p_first_grav->g_x = 15.0;
//  src_p_first_grav->g_y = 16.0;
//  src_p_second_grav->g_z = 17.0;

  int check = gravPack(dst_p_grav_first_half, dst_p_grav_second_half, src_p_first_grav, src_p_second_grav);

  CAN.sendMsgBuf(msgIDs->grav_first_half_ID, 0, 8, dst_p_grav_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
  CAN.sendMsgBuf(msgIDs->grav_second_half_ID, 0, 8, dst_p_grav_second_half);

}

/////////////////////////////////////////// SETUP FUNCTION //////////////////////////////////////////////////////////

void setup(void) {

  ////// Initializing BNO085 //////

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit BNO08x test!");

  if (!bno08x.begin_I2C()) {
    //    Serial.println("Failed to find BNO08x chip");

    while (1) {
      delay(10);
    }
  }
  //  Serial.println("BNO08x Found!");

  setReports(enbld_reports, reportIntervalUs);// setting report types and frequency

  //  Serial.println("Reading events");
  //  delay(100);

  ////// Initializing BNO055 //////
//  bno.begin()

  ////// CAN-SPI //////

  while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_10MHz)) {
    //        Serial.println("CAN init fail, retry..."); // initializing the CAN-SPI 3.3V click board (PAY ATTENTION: the datarate needs to be the same on the whole CAN bus)
    delay(10);
  }
}

/////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////

void loop() {
  ////// First, read reports from the BNO085 //////

  if (bno08x.wasReset()) { // if BNO085 was reset, set again the desired report types
    Serial.print("sensor was reset ");
    setReports(enbld_reports, reportIntervalUs);
  }
  //
  if (bno08x.getSensorEvent(&sensorValue)) {

    src_imu_heartbeat.imu_cal_status=sensorValue.status;
    sendImuHeartbeatCAN(dst_p_imu_heartbeat,&src_imu_heartbeat, &messageIDs);
    
    switch (sensorValue.sensorId) {

      case SH2_GYRO_INTEGRATED_RV:{
    
        src_p_quat_first_half.q_w = sensorValue.un.gyroIntegratedRV.real;
        src_p_quat_first_half.q_i = sensorValue.un.gyroIntegratedRV.i;
        src_p_quat_second_half.q_j = sensorValue.un.gyroIntegratedRV.j;
        src_p_quat_second_half.q_k = sensorValue.un.gyroIntegratedRV.k;
    
        if (is_quat != true) { // converting to euler angles (Y-P-R)
            quaternionToEuler(src_p_quat_first_half.q_w, src_p_quat_first_half.q_i, src_p_quat_second_half.q_j, src_p_quat_second_half.q_k, &src_p_eul_first_half, &src_p_eul_second_half, is_deg); // is_deg: true -> degrees, false -> radians
          }
    
         sendOrientationOverCAN(dst_p_quat_first_half, dst_p_quat_second_half, dst_p_eul_first_half, dst_p_eul_second_half,
                                 &src_p_quat_first_half, &src_p_quat_second_half, &src_p_eul_first_half, &src_p_eul_second_half,
                                 &messageIDs,
                                 is_quat);                        
        break;
      }
       case SH2_ARVR_STABILIZED_RV:{
    
         src_p_quat_first_half.q_w = sensorValue.un.arvrStabilizedRV.real;
         src_p_quat_first_half.q_i = sensorValue.un.arvrStabilizedRV.i;
         src_p_quat_second_half.q_j = sensorValue.un.arvrStabilizedRV.j;
         src_p_quat_second_half.q_k = sensorValue.un.arvrStabilizedRV.k;
        
         if (is_quat != true) { // converting to euler angles (Y-P-R)
           quaternionToEuler(src_p_quat_first_half.q_w, src_p_quat_first_half.q_i, src_p_quat_second_half.q_j, src_p_quat_second_half.q_k, &src_p_eul_first_half, &src_p_eul_second_half, is_deg); // is_deg: true -> degrees, false -> radians
         }
        
         sendOrientationOverCAN(dst_p_quat_first_half, dst_p_quat_second_half, dst_p_eul_first_half, dst_p_eul_second_half,
                                 &src_p_quat_first_half, &src_p_quat_second_half, &src_p_eul_first_half, &src_p_eul_second_half,
                                 &messageIDs,
                                 is_quat);                        
         break;
       }
        case SH2_ROTATION_VECTOR:{
        
          src_p_quat_first_half.q_w = sensorValue.un.rotationVector.real;
          src_p_quat_first_half.q_i = sensorValue.un.rotationVector.i;
          src_p_quat_second_half.q_j = sensorValue.un.rotationVector.j;
          src_p_quat_second_half.q_k = sensorValue.un.rotationVector.k;
    
          if (is_quat != true) { // converting to euler angles (Y-P-R)
            quaternionToEuler(src_p_quat_first_half.q_w, src_p_quat_first_half.q_i, src_p_quat_second_half.q_j, src_p_quat_second_half.q_k, &src_p_eul_first_half, &src_p_eul_second_half, is_deg); // is_deg: true -> degrees, false -> radians
          }
    
          sendOrientationOverCAN(dst_p_quat_first_half, dst_p_quat_second_half, dst_p_eul_first_half, dst_p_eul_second_half,
                                 &src_p_quat_first_half, &src_p_quat_second_half, &src_p_eul_first_half, &src_p_eul_second_half,
                                 &messageIDs,
                                 is_quat);
          break;
        }
        
        case SH2_GAME_ROTATION_VECTOR:{
        
          src_p_quat_first_half.q_w = sensorValue.un.gameRotationVector.real;
          src_p_quat_first_half.q_i = sensorValue.un.gameRotationVector.i;
          src_p_quat_second_half.q_j = sensorValue.un.gameRotationVector.j;
          src_p_quat_second_half.q_k = sensorValue.un.gameRotationVector.k;
    
          if (is_quat != true) { // converting to euler angles (Y-P-R)
            quaternionToEuler(src_p_quat_first_half.q_w, src_p_quat_first_half.q_i, src_p_quat_second_half.q_j, src_p_quat_second_half.q_k, &src_p_eul_first_half, &src_p_eul_second_half, is_deg); // is_deg: true -> degrees, false -> radians
          }
    
          sendOrientationOverCAN(dst_p_quat_first_half, dst_p_quat_second_half, dst_p_eul_first_half, dst_p_eul_second_half,
                                 &src_p_quat_first_half, &src_p_quat_second_half, &src_p_eul_first_half, &src_p_eul_second_half,
                                 &messageIDs,
                                 is_quat);
    
          break;
        }
        
        case SH2_GYROSCOPE_CALIBRATED:{
    
          src_p_gyro_first_half.gyro_x=sensorValue.un.gyroscope.x; //gyro (1000 Hz max)
          src_p_gyro_first_half.gyro_y=sensorValue.un.gyroscope.y;
          src_p_gyro_second_half.gyro_z=sensorValue.un.gyroscope.z; 
          src_p_gyro_second_half.imu_cal_status=sensorValue.status; // calibration status packed into gyro, since the second message has 4 bytes available
    
          sendGyroOverCAN(dst_p_gyro_first_half, dst_p_gyro_second_half,&src_p_gyro_first_half, &src_p_gyro_second_half, &messageIDs);

          break;
        }

        case SH2_RAW_GYROSCOPE:{
    
          src_p_gyro_first_half.gyro_x=sensorValue.un.rawGyroscope.x; //gyro (1000 Hz max)
          src_p_gyro_first_half.gyro_y=sensorValue.un.rawGyroscope.y;
          src_p_gyro_second_half.gyro_z=sensorValue.un.rawGyroscope.z; 
          src_p_gyro_second_half.imu_cal_status=sensorValue.status; // calibration status packed into gyro, since the second message has 4 bytes available
    
          sendGyroOverCAN(dst_p_gyro_first_half, dst_p_gyro_second_half,&src_p_gyro_first_half, &src_p_gyro_second_half, &messageIDs);

          break;
        }
        
        case SH2_GRAVITY:{
    
          src_p_grav_first_half.g_x = sensorValue.un.gravity.x;
          src_p_grav_first_half.g_y = sensorValue.un.gravity.y;
          src_p_grav_second_half.g_z = sensorValue.un.gravity.z;
    
          sendGravOverCAN(dst_p_grav_first_half, dst_p_grav_second_half, &src_p_grav_first_half, &src_p_grav_second_half, &messageIDs);
    
          break;
        }
        
        
     }
  }

}
