Example Brief:
===================
The example application demonstrates the use of the FXLS8974CF & MPL3115 sensor in the Normal (Non-Buffered) Mode.
The example demonstrates configuration of all registers reguired to put the sensor into Standard Mode and read out
Acceleration, Pressure and Temperature samples. LED status on FRDM-MCXA153 shows following:

GEEN LED Blinking: FXLS8974CF Accel Z axis +1/-1g
RED LED Blinking:  FXLS8974CF Accel X axis +1/-1g
BLUE LED Blinking: FXLS8974CF Accel Y axis +1/-1g

User Configuration: Choose MPL3115_MODE as either PRESSURE_MODE or ALTITUDE_MODE. Default is set to ALTITUDE_MODE.

Hardware requirements
===================
- Mini/micro C USB cable
- FRDM-MCXN947 board
- Accel&Pressure Click (https://www.mikroe.com/accelpressure-click)
- Personal Computer

Board settings
============
Connect Accel&Pressure MikroE click board to FRDM-MCXN947 MCU on MikroBUS header.

Prepare the Demo
===============
1.  Connect a USB cable between the host PC and the MCU-Link USB port on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Build the project.
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
===============
 When the demo runs (with MPL3115_MODE as ALTITUDE_MODE) successfully, you can see the Samples printed to the terminal.

 ISSDK FXLS8974CF & MPL3115 sensor driver example demonstration
 Successfully Initialized FXLS8974CF Sensor
 Successfully Initialized MPL3115 Sensor
 Successfully Applied FXLS8974 Sensor Configuration
 Successfully Applied MPL3115 Configuration for Altimeter mode

 Accel X    = -0.011 g
 Accel Y    =  0.004 g
 Accel Z    =  1.075 g
 Pressure   = 391 Pa
 Temerature = 24 degC

When the demo runs (with MPL3115_MODE as PRESSURE_MODE) successfully, you can see the Samples printed to the terminal.

 ISSDK FXLS8974CF & MPL3115 sensor driver example demonstration
 Successfully Initialized FXLS8974CF Sensor
 Successfully Initialized MPL3115 Sensor
 Successfully Applied FXLS8974 Sensor Configuration
 Successfully Applied MPL3115 Configuration for Pressure mode

 Accel X    = -0.011 g
 Accel Y    =  0.004 g
 Accel Z    =  1.075 g
 Pressure   = 96806 Pa
 Temerature = 24 degC
