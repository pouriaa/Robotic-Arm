# Robot Arm

The files within this repository contain the source code designed for an LP51 for use in a 2-DoF robotic arm.

## Parts
* HCTL2022 Integrated Circuit
* MCP3004 Integrated Circuit
* LP51 Board (with an 8051 microcontroller)
* 2x Potentiometers for master control
* 2x Optical encoders
* 2x DC Motors

## Summary
The source contains a PID which uses an ADC to convert two potentiometer values into a setpoint to be processed by software PIDs. The HCTL2022 is responsible for storing the number of revolutions given from each optical encoder. The DC motors are attached to the same arm mechanically and wired in to be controlled using PWM by the controller.

**NOTE:** The code should only be used as a reference for interfacing with an LP51, different mechanical and electrical designs will require further integration.