# STM32 IMU and Motor Control Firmware

## Overview
This firmware is designed for an STM32 microcontroller to perform the following core functions:

1. **IMU Data Acquisition and Processing:**
   - Captures raw data from an IMU - Adafruit LSM9DS1.
   - Processes the data using an AHRS (Attitude and Heading Reference System) algorithm to compute orientation.
   - Transmits the orientation data over the I2C interface using Direct Memory Access (DMA) for efficient and low-latency communication.

2. **Motor and Servo Control:**
   - Implements PID (Proportional-Integral-Derivative) control algorithms for accurate motor and servo actuation.
   - Continuously monitors and adjusts motor and servo positions to achieve desired motion profiles.
   - Communicates real-time position data over UART to facilitate integration with the ROS2 control hardware interface.

## Communication Interfaces
- **I2C (DMA Mode):** Utilized for efficient transmission of orientation data to a master controller.
- **UART:** Facilitates the transfer of motor and servo position data to the ROS2 control interface, enabling real-time feedback and control.

## Resources:
- [LSM9DS1 sensor driver](https://github.com/STMicroelectronics/lsm9ds1-pid)
- [Adafruit AHRS algorithm](https://github.com/adafruit/Adafruit_AHRS)
- [Serial communication and PID](https://github.com/joshnewans/ros_arduino_bridge)

