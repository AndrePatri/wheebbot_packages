/////////////////////////////////////////////// USEFUL DEFINITIONS ///////////////////////////////////////////////////

#define BNO08X_RESET -1 //BNO085 reset pin; I2C does not need it, so -1 is used

//// CAN msg related defs ////

/////////////////////////////////////////////// IMPORTED LIBRARIES ///////////////////////////////////////////////////

// Including all Arduino-related stuff
#include <Arduino.h>

// Using BNO08x IMU library:
#include <Adafruit_BNO08x.h>

// Using (modified) Seedstudio CAN library:
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>

// Importing the libraries used for packing/unpacking and coding/decoding IMU messages on the CAN bus (almost completely generated useing "cantools" Python utility)

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

///////////////////////////////////////////////  USED STRUCTURES  ////////////////////////////////////////////////////

// Structure to hold the orientation quaternion (if used)
struct orient_quat_t {  
  float q_w;
  float q_i;
  float q_j;
  float q_k;
} quat={0.0,0.0,0.0,0.0};

// Structure to hold the report settings; to enable/disable one or more reports, set the values to true/false (ATTENTION: the more reports you enable, the lower the update rate be)
struct enabled_reports {
  
  const bool IS_SH2_GYRO_INTEGRATED_RV=false; // Top frequency is reported to be 1000Hz (but freq is somewhat variable); potentially lessa accurate, but provides also a calibrated gyroscope reading
  const bool IS_SH2_ARVR_STABILIZED_RV=false; // top frequency is about 250Hz but this report is more accurate
  const bool IS_SH2_ROTATION_VECTOR=true; // Most accurate (uses gyro,accel and magn)
  const bool IS_SH2_GEOMAGNETIC_ROTATION_VECTOR=false; // no gyroscope (less power)
  const bool IS_SH2_GAME_ROTATION_VECTOR=false; // no magnetometer (no yaw jumps)
  
  const bool IS_SH2_ACCELEROMETER=false; // total acceleration
  const bool IS_SH2_GYROSCOPE_CALIBRATED=true;
  const bool IS_SH2_MAGNETIC_FIELD_CALIBRATED=false;
  const bool IS_SH2_LINEAR_ACCELERATION=false; // total acceleration - gravity
  const bool IS_SH2_GRAVITY=false;
  
  
  const bool IS_SH2_SHAKE_DETECTOR=false;
  const bool IS_SH2_PERSONAL_ACTIVITY_CLASSIFIER=false;
  
  const bool IS_SH2_STEP_COUNTER=false;
  const bool IS_SH2_STABILITY_CLASSIFIER=false;
  const bool IS_SH2_RAW_ACCELEROMETER=false;
  const bool IS_SH2_RAW_GYROSCOPE=false;
  const bool IS_SH2_RAW_MAGNETOMETER=false;
  
  
} enbld_reports;


////// IMU CAN message handling //////

struct whee_b_bot_imu_can_quat_first_half_t {
  
            float q_w;
            float q_i;
            
} src_p_quat_first_half={0.0,0.0};

struct whee_b_bot_imu_can_quat_second_half_t {
  
            float q_j;
            float q_k;
            
} src_p_quat_second_half={0.0,0.0};
///////////////////////////////////////////////  INITIALIZATIONS  ////////////////////////////////////////////////////

////// BNO085 //////

// IMU object initialization
Adafruit_BNO08x  bno08x(BNO08X_RESET);

// Structure which holds reports, IMU name, etc...
sh2_SensorValue_t sensorValue;

// Report rate (the BNO085 will try to keep a rate >= than 1/reportIntervalUs Hz)
long reportIntervalUs = 5000; // report rate; 2000Us->500Hz; 4000Us->250Hz; 5000Us->200Hz; 10000Us->100Hz; 20000Us->50Hz;40000Us->25Hz

////// CAN-SPI //////

const int SPI_CS_PIN = 10; // chip select pin
const int CAN_INT_PIN = 8; // interrupt pin
mcp2515_can CAN(SPI_CS_PIN);

////// IMU over CAN message encoding and packing utilities //////

double sensor_dt=1000000; // microseconds

unsigned long startMicros;  //some global variables available anywhere in the program
unsigned long currentMicros;

uint8_t dst_p_quat_first_half[8];// 8 byte data destination variable for CAN msgs
uint8_t dst_p_quat_second_half[8];// 8 byte data destination variable for CAN msgs

/////////////////////////////////////////// FUNCTION DEFINITIONS /////////////////////////////////////////////////////

// Function to set report types
void setReports(struct enabled_reports enbld_rep, long report_interval) {
  
  Serial.println("Setting desired reports");
  
  if (enbld_rep.IS_SH2_GYRO_INTEGRATED_RV==true){
      if (! bno08x.enableReport(SH2_GYRO_INTEGRATED_RV,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_ARVR_STABILIZED_RV==true){
      if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GEOMAGNETIC_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GAME_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_ACCELEROMETER==true){
      if (! bno08x.enableReport(SH2_ACCELEROMETER,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GYROSCOPE_CALIBRATED==true){
      if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_MAGNETIC_FIELD_CALIBRATED==true){
      if (! bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_LINEAR_ACCELERATION==true){
      if (! bno08x.enableReport(SH2_LINEAR_ACCELERATION,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GRAVITY==true){
      if (! bno08x.enableReport(SH2_GRAVITY,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GEOMAGNETIC_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_GAME_ROTATION_VECTOR==true){
      if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_RAW_ACCELEROMETER==true){
      if (! bno08x.enableReport(SH2_RAW_ACCELEROMETER,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_RAW_GYROSCOPE==true){
      if (! bno08x.enableReport(SH2_RAW_GYROSCOPE,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
  if (enbld_rep.IS_SH2_RAW_MAGNETOMETER==true){
      if (! bno08x.enableReport(SH2_RAW_MAGNETOMETER,report_interval)) { // Setting report type/types and report rate
        Serial.println("Could not enable stabilized remote vector");
      }
  }
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

int orient_quat_pack(
    uint8_t* dst_p_first,
    uint8_t* dst_p_second,
    const struct whee_b_bot_imu_can_quat_first_half_t* src_p_first,
    const struct whee_b_bot_imu_can_quat_second_half_t* src_p_second)
{
    uint32_t q_i;
    uint32_t q_w;
    uint32_t q_j;
    uint32_t q_k;
    
//    memset(&dst_p_first[0], 0, 8);

    memset(dst_p_first, 0, 8); // resetting dst

    memcpy(&q_w, &(src_p_first->q_w), sizeof(q_w));
    dst_p_first[0] |= pack_left_shift_u32(q_w, 0u, 0xffu);
    dst_p_first[1] |= pack_right_shift_u32(q_w, 8u, 0xffu);
    dst_p_first[2] |= pack_right_shift_u32(q_w, 16u, 0xffu);
    dst_p_first[3] |= pack_right_shift_u32(q_w, 24u, 0xffu);
    
    memcpy(&q_i, &(src_p_first->q_i), sizeof(q_i));
    dst_p_first[4] |= pack_left_shift_u32(q_i, 0u, 0xffu);
    dst_p_first[5] |= pack_right_shift_u32(q_i, 8u, 0xffu);
    dst_p_first[6] |= pack_right_shift_u32(q_i, 16u, 0xffu);
    dst_p_first[7] |= pack_right_shift_u32(q_i, 24u, 0xffu);
       
    memset(dst_p_second, 0, 8); // resetting dst
    
    memcpy(&q_j, &(src_p_second->q_j), sizeof(q_j));
    dst_p_second[0] |= pack_left_shift_u32(q_j, 0u, 0xffu);
    dst_p_second[1] |= pack_right_shift_u32(q_j, 8u, 0xffu);
    dst_p_second[2] |= pack_right_shift_u32(q_j, 16u, 0xffu);
    dst_p_second[3] |= pack_right_shift_u32(q_j, 24u, 0xffu);
    
    memcpy(&q_k, &(src_p_second->q_k), sizeof(q_k));
    dst_p_second[4] |= pack_left_shift_u32(q_k, 0u, 0xffu);
    dst_p_second[5] |= pack_right_shift_u32(q_k, 8u, 0xffu);
    dst_p_second[6] |= pack_right_shift_u32(q_k, 16u, 0xffu);
    dst_p_second[7] |= pack_right_shift_u32(q_k, 24u, 0xffu);
    
    return (8);
}

// send orientation quaternion over CAN
void sendOrientationOverCAN(uint8_t* dst_p_quat_first_half,uint8_t* dst_p_quat_second_half,
                            struct whee_b_bot_imu_can_quat_first_half_t*src_p_first_quat,struct whee_b_bot_imu_can_quat_second_half_t* src_p_second_quat) {
                                                     
       
//       src_p_first_quat->q_w=0.20373535; //0x3e50a000
//       src_p_first_quat->q_i=-0.07336426; //0xbd964006
       
       Serial.print("\n source q_w:\t"); Serial.print(src_p_first_quat->q_w,8); Serial.print("\t"); Serial.print("source q_i:\t"); Serial.print(src_p_first_quat->q_i,8); Serial.print("\n"); // checking if source package is read correctly
       Serial.print("First half HEX:\t"); Serial.print(dst_p_quat_first_half[0],HEX);Serial.print("-"); Serial.print(dst_p_quat_first_half[1],HEX);Serial.print("-");Serial.print(dst_p_quat_first_half[2],HEX);Serial.print("-");Serial.print(dst_p_quat_first_half[3],HEX);
       Serial.print("---");Serial.print(dst_p_quat_first_half[4],HEX);Serial.print("-"); Serial.print(dst_p_quat_first_half[5],HEX);Serial.print("-"); Serial.print(dst_p_quat_first_half[6],HEX);Serial.print("-"); Serial.print(dst_p_quat_first_half[7],HEX);Serial.print("-"); Serial.print(":\n");
       Serial.print("First half binary:\t"); Serial.print(dst_p_quat_first_half[0],BIN);Serial.print("-"); Serial.print(dst_p_quat_first_half[1],BIN);Serial.print("-");Serial.print(dst_p_quat_first_half[2],BIN);Serial.print("-");Serial.print(dst_p_quat_first_half[3],BIN);
       Serial.print("---");Serial.print(dst_p_quat_first_half[4],BIN);Serial.print("-"); Serial.print(dst_p_quat_first_half[5],BIN);Serial.print("-"); Serial.print(dst_p_quat_first_half[6],BIN);Serial.print("-"); Serial.print(dst_p_quat_first_half[7],BIN);Serial.print("-"); Serial.print(":\n");

       Serial.print("\n source q_j:\t"); Serial.print(src_p_second_quat->q_j,8); Serial.print("\t"); Serial.print("source q_k:\t"); Serial.print(src_p_second_quat->q_k,8); Serial.print("\n"); // checking if source package is read correctly
       Serial.print("Second half HEX:\t"); Serial.print(dst_p_quat_second_half[0],HEX);Serial.print("-"); Serial.print(dst_p_quat_second_half[1],HEX);Serial.print("-");Serial.print(dst_p_quat_second_half[2],HEX);Serial.print("-");Serial.print(dst_p_quat_second_half[3],HEX);
       Serial.print("---");Serial.print(dst_p_quat_second_half[4],HEX);Serial.print("-"); Serial.print(dst_p_quat_second_half[5],HEX);Serial.print("-"); Serial.print(dst_p_quat_second_half[6],HEX);Serial.print("-"); Serial.print(dst_p_quat_second_half[7],HEX);Serial.print(":\n");
       Serial.print("Second half binary:\t"); Serial.print(dst_p_quat_second_half[0],BIN);Serial.print("-"); Serial.print(dst_p_quat_second_half[1],BIN);Serial.print("-");Serial.print(dst_p_quat_second_half[2],BIN);Serial.print("-");Serial.print(dst_p_quat_second_half[3],BIN);
       Serial.print("---");Serial.print(dst_p_quat_second_half[4],BIN);Serial.print("-"); Serial.print(dst_p_quat_second_half[5],BIN);Serial.print("-"); Serial.print(dst_p_quat_second_half[6],BIN);Serial.print("-"); Serial.print(dst_p_quat_second_half[7],BIN);Serial.print(":\n");

       int check=orient_quat_pack(dst_p_quat_first_half,dst_p_quat_second_half,src_p_first_quat,src_p_second_quat);
       
       CAN.sendMsgBuf(0x01, 0, 8, dst_p_quat_first_half);

       CAN.sendMsgBuf(0x02, 0, 8, dst_p_quat_second_half);
      
      }

/////////////////////////////////////////// SETUP FUNCTION //////////////////////////////////////////////////////////

void setup(void) {
  
  ////// It is not possible to set a sensor update < report interval ////// 

  if (sensor_dt<reportIntervalUs){sensor_dt=reportIntervalUs;}

  ////// Initializing BNO085 //////
  
  Serial.begin(115200);
//  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
//  Serial.println("Adafruit BNO08x test!");
  
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(enbld_reports, reportIntervalUs);// setting report types and frequency

//  Serial.println("Reading events");
//  delay(100);

  ////// CAN-SPI //////

  while (CAN_OK != CAN.begin(CAN_1000KBPS,MCP_10MHz)) {  
        Serial.println("CAN init fail, retry..."); // initializing the CAN-SPI 3.3V click board (PAY ATTENTION: the datarate needs to be the same on the whole CAN bus)
        delay(100);
  }
  
}

/////////////////////////////////////////// (first) LOOP FUNCTION -->  handles IMU reading //////////////////////////////////////////////////////////

void loop() {
  
  ////// First, read reports from BNO085 //////
  
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(enbld_reports,reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {

    src_p_quat_first_half.q_w=sensorValue.un.rotationVector.real; //rotationVector (most accurate, 400 Hz max)
    src_p_quat_first_half.q_i=sensorValue.un.rotationVector.i;
    src_p_quat_second_half.q_j=sensorValue.un.rotationVector.j; //rotationVector (most accurate, 400 Hz max)
    src_p_quat_second_half.q_k=sensorValue.un.rotationVector.k;

    sendOrientationOverCAN(dst_p_quat_first_half,dst_p_quat_second_half,
                            &src_p_quat_first_half,&src_p_quat_second_half);
    
  }
  
//  currentMicros = micros();  //get the current "time" (actually the number of milliseconds since the program started)
//  if (currentMicros - startMicros >= sensor_dt)  //test whether the period has elapsed
//  {
//    Serial.print("Delta dt:\t");  Serial.print(currentMicros - startMicros);                Serial.print("\t");
//    
//    Serial.print("Cal_stat:\t");   Serial.print(sensorValue.status);        Serial.print("\t");  // This is accuracy in the range of 0 to 3
//  
//    Serial.print("q_w:\t");        Serial.print(quat.q_w,6);                Serial.print("\t");
//    Serial.print("q_i:\t");        Serial.print(quat.q_i,6);                Serial.print("\t");            
//    Serial.print("q_j:\t");        Serial.println(quat.q_j,6);              Serial.print("\t");            
//    Serial.print("q_k:\t");        Serial.println(quat.q_k,6);              Serial.print("\n");
    
//    Serial.print("gyro_x:\t");     Serial.print(gyro.gyro_x,6);             Serial.print("\t");
//    Serial.print("gyro_y:\t");     Serial.print(gyro.gyro_y,6);             Serial.print("\t");            
//    Serial.print("gyro_z:\t");     Serial.println(gyro.gyro_z,6);           Serial.print("\t");
    
    ////// Sending messages on the CAN bus//////
  
//    sendOrientationOverCAN(&dst_p_quat_first_half,&dst_p_quat_second_half,
//                            &src_p_quat_first_half,&src_p_quat_second_half);
//                            
//    startMicros = currentMicros;  //IMPORTANT to save the start time of the current LED state.
//  }
}
