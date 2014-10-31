GCrash
======

  A small PIC12F1840   will be used to get  the acceleration data from the MPU6050 and transfer it via the bluetooth HC04 device.

  The transfer rate is 500 samples/sec  but could be increase to 1000 ifthe baud rate is set from 115200 to 230400. 


  This is a sample program to demonstrate how to implement a small cpu to transfer data.

  
  Single ascii command send via a terminal will control the system.

   the commands are,

    - [esc]  			    IDLE MODE   Do nothing.
    - G , g or [enter]		READY MODE  Wait for free fall.
    - D  or d				DROP MODE   Force system to trigger the drop.
    - H  or h				HIT MODE    Force system to trigger the crash.
    - I  or i         		INFO MODE   Display Acceleration data in G unit. (this is slow ~4ms)
    - ?                                 Display the release version.  (Idle mode only)
    - V or  v                           Display the battery voltage.  (Idle mode only)

  The READY mode will automatically go to DROP mode if all G values are smaller than 0.5 G and
  from DROP to HIT if one of the G value is bigger than 1.5 G.

   Files Information

   Main CPU program
   
    - GCrash.hex      This is the Hex file needed to burn the program into the cpu.
  
      
   The source code
 
    - GCrash.c        This is the PIC program written in C.
    - i2cMaster.c     This is the I2C code to communicate with the MPU6050.
    - i2cMaster.h     This is the I2C header file.
    - MPU6050.c       This is the MPU6050 code to access the accelerometer.
    - MPU6050.h       This is the MPU6050 header file.
 
   Schematic
   
    - GCrash_Schema.png  	Initial schema to test the cpu directly on the Raspberry Pi.
    - GCrash_Schema2.png 	Electronic schematic of the system.
    
   license
   
    - gpl.txt         GNU GENERAL PUBLIC LICENSE
    
