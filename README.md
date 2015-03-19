# Interrupt-At-Acceleration-Threshold--MPU-9150
Configure Low Power Accelerometer for Motion Interrupts

Microcontroller: TM4C123GH6PM by TI.

IDE: Code Composer Studio 6.

Sensor: MPU-9150 by Invensense.

This project is a sample code to set up an interrupt based on a user-programmed threshold value.
The user sets the acceleration threshold in SI unit (m/s^2) and the amout of time the sample should stay in order to be a valid reading, usefult to avoid errors and unintended impulses.
The interrupt comes through Pin B2, the sample code just reads the sensors data, however further actions could be taken at the interrupt.

The function called from MAIN is the main function forming the skeleton for this project, and used independently in other projects.
