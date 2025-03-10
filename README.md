# SplitFile_RcTest
Attempting to learn to split a 400 line long script into separate headers and cpp files. 


DOCUMENTATION:

folders:
        Lib - Folder containing libraries 
            HeadersAndCPPs - Folder containing headers and cpps called by main
        TeensyTestCode - functional examples of code that does in fact work and has been tested and vetted to work.

files:

        main.cpp - the main code compiling all headers and cpps
        platformio.ini - configuration for the microcontroller, don't touch this or you will break EVERYTHING


Libraries:
        HAL: lets us control pin qualities more in-depth i think
        AUTOPID: pid tuning library 
        ODriveArduino / ASCII / UART - libraries to setup communication to the ODrive , a servo motor controller for our steering gearbox
        VescUart - library to control our drive motor controllers
        SBUS - library to interface with an RC plane receiver
