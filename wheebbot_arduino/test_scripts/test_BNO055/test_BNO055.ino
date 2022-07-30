
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000, time_stamp=-1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    time_stamp=event->timestamp;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    time_stamp=event->timestamp;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    time_stamp=event->timestamp;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    time_stamp=event->timestamp;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    time_stamp=event->timestamp;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    time_stamp=event->timestamp;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x,5);
  Serial.print(" |\ty= ");
  Serial.print(y,5);
  Serial.print(" |\tz= ");
  Serial.print(z,5);
  Serial.print(" |\ttime_stamp: ");
  Serial.println(time_stamp,10);

}
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value,5); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value,5); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution,5); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
  Serial.println();
}



struct euler_t{
  float yaw;
  float pitch;
  float roll;
} euler={0.0,0.0,0.0};

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void setup() {
 
  Serial.begin(115200);
  Serial.println("BNO055 tests"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
 
  bno.setMode(bno.OPERATION_MODE_NDOF);//sets the 9 DOF op mode

  bno.setExtCrystalUse(true); //recommended for best performance

}

void loop()
{
  sensors_event_t  angVelocityData , linearAccelData, gravityData;
  imu::Quaternion quat_reading=bno.getQuat();
  
  double time_stamp=millis();
  
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  double w=quat_reading.w();
  double i=quat_reading.x();
  double j=quat_reading.y();
  double k=quat_reading.z();

  quaternionToEuler(w, i, j, k, &euler,true);
  
  Serial.print(" |\tyaw: ");
  Serial.print(euler.yaw,5);
  Serial.print(" |\tpitch: ");
  Serial.print(euler.pitch,5);
  Serial.print(" |\troll: ");
  Serial.print(euler.roll,5);
  Serial.print(" |\ttime_stamp: ");
  Serial.println(time_stamp,10);

  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();

  displayCalStatus();

  delay(100);
}
