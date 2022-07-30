/////////////////////////////////////////////// MAIN WITHOUT CALLS TO SERIAL (they slow down code and are not needed on the final version of the prototype) ///////////////////////////////////////////////////

// PERFORMANCE (Nano Iot) depending on the chosen orientation report type:
// SH2_GYRO_INTEGRATED_RV ---> interval between Heartbeats is approx. 7-8 ms (peaks of 10 ms are possible)
// SH2_ARVR_STABILIZED_RV ---> interval between Heartbeats is approx. 10 ms
// SH2_ROTATION_VECTOR ---> interval between Heartbeats is approx. 10 ms
// SH2_GEOMAGNETIC_ROTATION_VECTOR ---> interval between Heartbeats is approx. 10 ms
// SH2_GAME_ROTATION_VECTOR ---> interval between Heartbeats is approx. 10 ms

/////////////////////////////////////////////// USEFUL DEFINITIONS ///////////////////////////////////////////////////

#define BNO08X_RESET -1 //BNO085 reset pin; I2C does not need it, so -1 is used

/////////////////////////////////////////////// IMPORTED LIBRARIES ///////////////////////////////////////////////////

// Including all Arduino-related stuff:

#include <Arduino.h>

// BNO08x IMU library:

#include <Adafruit_BNO08x.h>

// Using (modified) Seedstudio CAN library:
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>

// BNO055 library (if used):

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Importing the libraries used for packing/unpacking and coding/decoding IMU messages on the CAN bus (may be already included by Arduino.h):

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

///////////////////////////////////////////////  USED STRUCTURES  ////////////////////////////////////////////////////

// Report rates (the BNO085 will: 0.9 * RequestedRate <= ConfiguredRate <= 2.1 * RequestedRate)
struct IMU_report_dts {
  
  long GYRO_INTEGRATED_RV_dt;
  long ARVR_STABILIZED_RV_dt;
  long ROTATION_VECTOR_dt;
  long GEOMAGNETIC_ROTATION_VECTOR_dt;
  long GAME_ROTATION_VECTOR_dt;
  
} reportDts={5000,5000,5000,5000,5000};

// Structure to hold the BNO085 report settings; to enable/disable one or more reports, set the values to true/false (ATTENTION: enable only one orientation report at the same time)
struct enabled_reports {

  const bool IS_SH2_GYRO_INTEGRATED_RV = false; // Top frequency is reported to be 1000Hz (but freq is somewhat variable); potentially less accurate, but provides also a calibrated gyroscope reading
  const bool IS_SH2_ARVR_STABILIZED_RV = false; // top frequency is about 250Hz but this report is more accurate
  const bool IS_SH2_ROTATION_VECTOR = false; // Most accurate (uses gyro,accel and magn)
  const bool IS_SH2_GEOMAGNETIC_ROTATION_VECTOR = false; // no gyroscope (less power)
  const bool IS_SH2_GAME_ROTATION_VECTOR = true; // no magnetometer (no yaw jumps)

} enbld_reports;

struct message_IDs {
  
  unsigned long imu_status_ID = 0x000;
  unsigned long quat_first_half_ID = 0x001;
  unsigned long quat_second_half_ID = 0x002;
  unsigned long eul_first_half_ID = 0x003;
  unsigned long eul_second_half_ID = 0x004;
  unsigned long gyro_first_half_ID = 0x005;
  unsigned long gyro_second_half_ID = 0x006;
  unsigned long lin_acc_first_half_ID = 0x007;// not used here, but present in the CAN database
  unsigned long lin_acc_second_half_ID = 0x008;// not used here, but present in the CAN database
  unsigned long grav_first_half_ID = 0x009;
  unsigned long grav_second_half_ID = 0x00A;
  unsigned long raw_gyro_first_half_ID = 0x00B;// not used here, but present in the CAN database
  unsigned long raw_gyro_second_half_ID = 0x00C;// not used here, but present in the CAN database
  unsigned long raw_acc_first_half_ID = 0x00D;// not used here, but present in the CAN database
  unsigned long raw_acc_second_half_ID = 0x00E;// not used here, but present in the CAN database

} messageIDs;


////// IMU CAN message handling //////

struct imu_status_t {

  uint8_t orient_cal_status085;

  uint8_t orient_cal_status055;
  
  int8_t temperature;

} src_imu_heartbeat = {0,0,0};

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

} src_p_gyro_second_half = {0.0};

static struct imu_grav_first_half_t {

  float g_x;
  float g_y;

} src_p_grav_first_half = {0.0, 0.0};

struct imu_grav_second_half_t {

  float g_z;

} src_p_grav_second_half = {0.0};

///////////////////////////////////////////////  INITIALIZATIONS  ////////////////////////////////////////////////////

//////////////////  IMU-related stuff  //////////////////

////// BNO085 //////

// IMU object initialization
Adafruit_BNO08x  bno08x(BNO08X_RESET);

// Structure which holds reports, IMU name, etc...
sh2_SensorValue_t sensorValue;

bool is_quat = false; // whether of not to use quaternions or euler engles
uint32_t is_deg = 0; // if using euler angles, whether or not to use degrees
//const float G=9.80665; //gravity acceleration

////// BNO055, if used (to provide calibrated gyroscope and acceleration data) //////

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //sensoID, I2C address

sensors_event_t  bno055Gyro;

////// CAN-SPI //////

const int SPI_CS_PIN = 10; // chip select pin
const int CAN_INT_PIN = 8; // interrupt pin
mcp2515_can CAN(SPI_CS_PIN);

////// IMU over CAN message encoding and packing utilities //////

// BNO085 (used for orientation)
uint8_t dst_p_imu_heartbeat[8];// 8 byte data destination variable for CAN msgs
uint8_t dst_p_quat_first_half[8];
uint8_t dst_p_quat_second_half[8];
uint8_t dst_p_eul_first_half[8];
uint8_t dst_p_eul_second_half[8];

// BNO055 (used for the rest)
uint8_t dst_p_gyro_first_half[8];
uint8_t dst_p_gyro_second_half[8];
uint8_t dst_p_lin_acc_first_half[8];
uint8_t dst_p_lin_acc_second_half[8];

/////////////////////////////////////////// FUNCTION DEFINITIONS /////////////////////////////////////////////////////

void (* resetFunc) (void) = 0; //declare a reset function @ address 0

// Function to set report types
void bno085SetReports(struct enabled_reports enbld_rep, struct IMU_report_dts reportDts) {

  Serial.println("Setting desired prientation reports");

  if (enbld_rep.IS_SH2_GYRO_INTEGRATED_RV == true) {
    bno08x.enableReport(SH2_GYRO_INTEGRATED_RV,reportDts.GYRO_INTEGRATED_RV_dt); // Setting report type/types and report rate
  }
  if (enbld_rep.IS_SH2_ARVR_STABILIZED_RV == true) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV,reportDts.ARVR_STABILIZED_RV_dt); // Setting report type/types and report rate
  }
  if (enbld_rep.IS_SH2_ROTATION_VECTOR == true) {
    bno08x.enableReport(SH2_ROTATION_VECTOR,reportDts.ROTATION_VECTOR_dt); // Setting report type/types and report rate
    }
  if (enbld_rep.IS_SH2_GEOMAGNETIC_ROTATION_VECTOR == true) {
    bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,reportDts.GEOMAGNETIC_ROTATION_VECTOR_dt); // Setting report type/types and report rate
    }
  if (enbld_rep.IS_SH2_GAME_ROTATION_VECTOR == true) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,reportDts.GAME_ROTATION_VECTOR_dt); // Setting report type/types and report rate 
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
uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

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
  uint32_t temp;

  memset(dst_p, 0, 8);
  
  dst_p[0] |= pack_left_shift_u8(src_p->orient_cal_status085, 0u, 0xffu);
  dst_p[1] |= pack_left_shift_u8(src_p->orient_cal_status055, 0u, 0xffu);
  dst_p[2] |= pack_left_shift_u8(src_p->temperature, 0u, 0xffu);
  
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


  int check = gyroPack(dst_p_gyro_first_half, dst_p_gyro_second_half, src_p_first_gyro, src_p_second_gyro);

  CAN.sendMsgBuf(msgIDs->gyro_first_half_ID, 0, 8, dst_p_gyro_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
  CAN.sendMsgBuf(msgIDs->gyro_second_half_ID, 0, 8, dst_p_gyro_second_half);

}

void sendGravOverCAN(uint8_t *dst_p_grav_first_half, uint8_t *dst_p_grav_second_half,
                     struct imu_grav_first_half_t *src_p_first_grav, struct imu_grav_second_half_t *src_p_second_grav,
                     struct message_IDs *msgIDs) {

  int check = gravPack(dst_p_grav_first_half, dst_p_grav_second_half, src_p_first_grav, src_p_second_grav);

  CAN.sendMsgBuf(msgIDs->grav_first_half_ID, 0, 8, dst_p_grav_first_half);// args:(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
  CAN.sendMsgBuf(msgIDs->grav_second_half_ID, 0, 8, dst_p_grav_second_half);

}
/////////////////////////////////////////// SETUP FUNCTION //////////////////////////////////////////////////////////

void setup(void) {
  
  ////// Initializing BNO085 //////
  
  if (!bno08x.begin_I2C())
  {
    while (1){delay(10);}
  }
  
  bno085SetReports(enbld_reports, reportDts);// setting report types and frequency

  ////// Initializing BNO055 --> used to obtain calibrated gyroscope readings //////
  
  if (!bno.begin())
  {
    while (1){delay(10);}
  }
  
  bno.setMode(bno.OPERATION_MODE_NDOF);//sets the 9 DOF op mode
  bno.setExtCrystalUse(true); //recommended for best performance
  
  ////// CAN-SPI //////

  while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_10MHz)) {
    delay(10);
  }
}

/////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////

void loop() {
  ////// First, read reports from the BNO085 //////

  if (bno08x.wasReset()) { // if BNO085 was reset, set again the desired report types
    bno085SetReports(enbld_reports, reportDts);
  }
  //
  if (bno08x.getSensorEvent(&sensorValue)) {

    // Building the sources for the Heartbeat message and sending it
    
    uint8_t system, gyro_cal, accel_cal, mag_cal = 0; // 0-3
    
    src_imu_heartbeat.orient_cal_status085=sensorValue.status;
    bno.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);
    src_imu_heartbeat.orient_cal_status055=gyro_cal;
    src_imu_heartbeat.temperature = bno.getTemp();
    
    sendImuHeartbeatCAN(dst_p_imu_heartbeat,&src_imu_heartbeat, &messageIDs);
    
    // Reading and sending BNO085 report
    
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

        case SH2_GEOMAGNETIC_ROTATION_VECTOR:{
        
          src_p_quat_first_half.q_w = sensorValue.un.geoMagRotationVector.real;
          src_p_quat_first_half.q_i = sensorValue.un.geoMagRotationVector.i;
          src_p_quat_second_half.q_j = sensorValue.un.geoMagRotationVector.j;
          src_p_quat_second_half.q_k = sensorValue.un.geoMagRotationVector.k;
    
          if (is_quat != true) { // converting to euler angles (Y-P-R)
            quaternionToEuler(src_p_quat_first_half.q_w, src_p_quat_first_half.q_i, src_p_quat_second_half.q_j, src_p_quat_second_half.q_k, &src_p_eul_first_half, &src_p_eul_second_half, is_deg); // is_deg: true -> degrees, false -> radians
          }
    
          sendOrientationOverCAN(dst_p_quat_first_half, dst_p_quat_second_half, dst_p_eul_first_half, dst_p_eul_second_half,
                                 &src_p_quat_first_half, &src_p_quat_second_half, &src_p_eul_first_half, &src_p_eul_second_half,
                                 &messageIDs,
                                 is_quat);
    
          break;
        }
        
     }
   
     
     // Reading and sending BNO05 report/s

     // (hopefully calibrated) gyroscope readings from bno055

     bno.getEvent(&bno055Gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
     
     src_p_gyro_first_half.gyro_x=bno055Gyro.gyro.x;
     src_p_gyro_first_half.gyro_y=bno055Gyro.gyro.y;
     src_p_gyro_second_half.gyro_z=bno055Gyro.gyro.z;
     
     sendGyroOverCAN(dst_p_gyro_first_half, dst_p_gyro_second_half,&src_p_gyro_first_half, &src_p_gyro_second_half, &messageIDs);

     

  }
}
