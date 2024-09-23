# UBLOX_ZED_F9P_UBX_Protocol_Driver

### I have successfully analyzed the UBX protocol and message creation logic for the UBLOX ZED-F9P GNSS module. Using the F9P-Interface Description, I wrote a configuration driver in C.

#### SerialCom.h and SerialCom.c: These files contain auxiliary communication tools and the necessary functions to send messages to the GNSS module. 
#### Driver.h and Driver.c: These source and header files are created to handle various operations such as forming and sending messages using the UBX protocol. They make up the configuration file.

#### This project can be adapted and used according to its purpose. It can be integrated into any microcontroller, allowing the necessary configuration settings to be made via this driver without relying on the U-Center software.
#### The project has been implemented in the C programming language.
