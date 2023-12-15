# Robotic Arm Control with Image Processing

## Introduction

This GitHub repository hosts a comprehensive robotic arm control system that utilizes image processing techniques for object detection and localization. The system is designed to control a two-link robotic arm, interfacing with various hardware components such as servos, a stepper motor, and a suction device. The integration of computer vision allows the arm to interact with and manipulate objects based on real-time camera input.

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
