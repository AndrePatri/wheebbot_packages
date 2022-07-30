# Arduino-related stuff, including the final script/s to be deployed on the microcontroller

In particular:

 - Test scripts (CAN-SPI board, IMU, etc ....)
 
 - Used libraries (modified Seedstudio CAN library)
 
 - Prototype scripts:
 
   - main &rarr; full main, including debug prints to serial (not to be used on the prototype)
   
   - main_fast &rarr; no serial prints, full info report 
   
   - main_faster &rarr; only essential reports (gyro and orientation), fastest
