# THIS FOLDER THE USED CUSTOM LIBRARIES; install them from the Arduino IDE and import them in your script

Some potentially useful notes:

 - Since the CAN-SPI click 3.3V has a 10MHz oscillator, which is rarely used, CAN libraries have to be modified to include all the bit timing settings for 10MHz oscillators

 - How to convert generated c files from cantools as a library:

      - generate .c and .h files with "cantools generate_c_source <database_name>.dbc"

      - create a library folder and a src folder inside it

      - copy the generated files into the src folder

      - change .c extension with .cpp

      - insert the code between the end of the "#define ..." block and the "#ifdef __cplusplus" inside a class:

         ```class class_name{
           public:
            class_name();
          };```
 
      - insert the declarations of the "pack_left/right_shift" functions (present in the .cpp file) into the .h file (removing the return)

      - add <class_name>::function_name, so that methods inside the class are defined

      - compress the folder (.zip) and importing through the IDE (or CLI)


 - How to use the generated methods-> Logic sequence:

      - encode messages
      - create message structure instance
      - call the pack function
      - send the obtained packet on the BUS
      - read the message 
      - unpack the message
      - decode the obtained strucure

 - BNO085 report types:

       - SH2_RAW_ACCELEROMETER
       - SH2_ACCELEROMETER
 	   - SH2_LINEAR_ACCELERATION
 	   - SH2_GRAVITY
 	   - SH2_RAW_GYROSCOPE
 	   - SH2_GYROSCOPE_CALIBRATED
 	   - SH2_GYROSCOPE_UNCALIBRATED
 	   - SH2_RAW_MAGNETOMETER
 	   - SH2_MAGNETIC_FIELD_CALIBRATED
 	   - SH2_MAGNETIC_FIELD_UNCALIBRATED
 	   - SH2_ROTATION_VECTOR
 	   - SH2_GAME_ROTATION_VECTOR
 	   - SH2_GEOMAGNETIC_ROTATION_VECTOR
 	   - SH2_PRESSURE
 	   - SH2_AMBIENT_LIGHT
 	   - SH2_HUMIDITY
 	   - SH2_PROXIMITY
 	   - SH2_TEMPERATURE
 	   - SH2_RESERVED
 	   - SH2_TAP_DETECTOR
 	   - SH2_STEP_DETECTOR
 	   - SH2_STEP_COUNTER
 	   - SH2_SIGNIFICANT_MOTION
 	   - SH2_STABILITY_CLASSIFIER
 	   - SH2_SHAKE_DETECTOR
 	   - SH2_FLIP_DETECTOR
       - SH2_PICKUP_DETECTOR
 	   - SH2_STABILITY_DETECTOR
 	   - SH2_PERSONAL_ACTIVITY_CLASSIFIER
 	   - SH2_SLEEP_DETECTOR
 	   - SH2_TILT_DETECTOR
 	   - SH2_POCKET_DETECTOR
 	   - SH2_CIRCLE_DETECTOR
 	   - SH2_HEART_RATE_MONITOR

 	   - SH2_ARVR_STABILIZED_RV 
	   - SH2_ARVR_STABILIZED_GRV 
	   - SH2_GYRO_INTEGRATED_RV 

	   - SH2_IZRO_MOTION_REQUEST
            
