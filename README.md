# ME-35 Final Project: Autonomous Operation Robot

## Introduction



## Other Folders

There are 2 other folders contained within this GitHub repository called "development_programs" and "setup_programs." Here is a short explanation of both: 

### 1. Development Programs

This folder contains 3 files (ikrangetesting.py, servo_stepper_init.py, surgical_robot_hardcode.py). These were used in the beginning of our process to determine the robots limits, initlialize the stepper, and make sure it could hit hardcoded points using inverse kinematics. 

### 2. Setup Programs

This folder contains 2 filels (cv_calibration.py, test_hardware.py). These were used before running the main code in order to make sure the camera was identifying the correct objects and that all motors and suction were properly plugged in. Use for debugging. 

## Features

### 1. Inverse Kinematics (File: `inverse_kinematics.py`)

The `inverse_kinematics.py` script provides functions for calculating the inverse kinematics of the robotic arm. Key features include:

- Forward kinematics calculation.
- Inverse kinematics calculation, considering joint angle limits.

### 2. Robotic Arm Control (File: `robotic_arm_control.py`)

The `robotic_arm_control.py` script serves as the main control program for the robotic arm and incorporates the following features:

- Servo control for the base and elbow joints of the robotic arm.
- Stepper motor control for linear motion.
- Suction device control for picking up and releasing objects.
- Wi-Fi connectivity for remote control.
- MQTT communication for subscribing to control topics.

### 3. Computer Vision (File: `computer_vision.py`)

The `computer_vision.py` script integrates computer vision techniques to enhance the robotic arm's functionality. Features include:

- Real-time camera feed processing for object detection.
- Calculation of real-world coordinates of detected objects.
- MQTT communication to publish object coordinates for robotic arm control.

## Getting Started

To set up and run the robotic arm system, follow these steps:

1. Install the required libraries and dependencies:

    ```bash
    pip install numpy opencv-python paho-mqtt
    ```

2. Connect the robotic arm components (servos, stepper motor, suction device) to your hardware platform (e.g., Raspberry Pi).

3. Configure Wi-Fi settings and MQTT broker information in the `mysecrets.py` and `myprojectsecrets.py` files.

4. Run the `robotic_arm_control.py` script:

   ```bash
   python robotic_arm_control.py
