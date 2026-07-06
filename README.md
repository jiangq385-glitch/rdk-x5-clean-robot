# Clean Robot Important Code

This package contains the main code used by the embodied intelligent home/restaurant cleaning robot.

## Directory layout

- `ros2_upper_controller/`
  - ROS2 upper-computer code running on RDK X5.
  - `my_ros/src/` contains robot perception, navigation, bringup, robot state, skills, messages, maps, and robot description packages.
  - `ros2_ws/src/` contains voice interaction, wake word, ASR/TTS, and AI gateway packages.
- `gd32_lower_controller/`
  - GD32F407 lower-computer firmware project.
  - `APPLICATION/` contains task logic, chassis control, gimbal/arm-related application code, and communication exchange code.
  - `BSP/` contains CAN, UART, PWM, SPI, delay, FIFO, and message drivers.
  - `MODULES/` contains servo, motor, PID, LCD, IMU, encoder, and other device modules.
  - `FreeRTOS/`, `CMSIS/`, `Library/`, and `Startup/` contain RTOS, MCU library, and startup support files.
  - `project/` keeps the Keil project files.

## Packaging notes

Generated files and local-only files were removed from this upload package, including `.git`, `.vscode`, ROS build/install/log folders, Python cache files, Keil `Objects`, `Listings`, debug output, and local J-Link/user UI files.

The retained files are source code, configuration, launch files, messages, maps, robot models, firmware source, and project files needed to show the implementation.
