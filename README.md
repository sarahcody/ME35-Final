```markdown
# Autonomous Operation Robot - ME-35 Final Project

## Overview

This repository contains the code for an autonomous operation robot developed as part of the ME-35 (Introduction to Mechatronics) final project. The project involves computer vision, robotics, and control systems to enable the robot to identify game pieces' coordinates and autonomously move to specific locations on a board.

### Primary Programs

1. **mysimpleik.py**

   - **Summary:**
     - A custom library providing functions for forward and inverse kinematics calculations, used in the robotic control system.

   - **Functions:**
     - `fwdkin(angles=[0,0], L=[14.5, 14.8])`: Calculate forward kinematics to determine end-effector coordinates.
     - `invkin(coords=[12, 12], L=[14.5, 14.8], servo_1_lim=[10, 150], servo_2_lim=[32, 120])`: Calculate inverse kinematics to determine servo angles.

   - **Dependencies:**
     - `numpy`
     - `math`

2. **surgical_robot_main.py**

   - **Summary:**
     - The main control program that utilizes computer vision to identify game piece coordinates and sends movement commands to the robot through MQTT.

   - **Functions:**
     - `mqtt_setup()`: Set up and connect to the MQTT broker.
     - `get_frame(vid)`: Capture and return a frame from the camera.
     - `process_image(frame, real_length, pixel_length, thresh_area)`: Process the frame to identify game piece coordinates.
     - `check_board(vid, thresh_area)`: Check if a game piece is present on the board.
     - `get_coords(vid, pixel_length, real_length, thresh_area, es=0.1, dt=0.1, i_timeout=20)`: Get precise coordinates of the identified game piece.
     - `send_data(client, topic, message)`: Publish data to the MQTT broker.
     - `move_stepper(client, pos)`: Move the stepper motor to a specified position.
     - `move_servo(client, init_pos, end_pos, step_size=1, reverse=False)`: Move the servo to a specified position.
     - `locate_part()`: Locate the game piece and return its coordinates.
     - `success_indication(client)`: Provide indication of a successful operation.

   - **Dependencies:**
     - `cv2` (OpenCV)
     - `numpy`
     - `math`
     - `time`
     - `mysimpleik` (Custom Inverse Kinematics module)
     - `paho.mqtt.client`
     - `myprojectsecrets` (Custom module for storing MQTT broker information)

3. **pico_code.py**

   - **Summary:**
     - MicroPython script designed for the Raspberry Pi Pico microcontroller. It acts as the interface between hardware components and MQTT, receiving commands and controlling the robot's movements.

   - **Functions:**
     - `setup()`: Initialize hardware components.
     - `connect_wifi(wifi)`: Connect to a Wi-Fi network.
     - `move_stepper(message)`: Move the stepper motor based on received MQTT messages.
     - `move_base(angle)`: Move the base servo to a specified angle.
     - `move_elbow(angle)`: Move the elbow servo to a specified angle.
     - `control_suction(message)`: Control the suction mechanism based on received MQTT messages.
     - `whenCalled(topic, msg)`: Callback function for processing MQTT messages.
     - `main()`: Main function for handling MQTT communication and controlling the robot.

   - **Dependencies:**
     - `machine` (MicroPython machine module)
     - `time` (MicroPython time module)
     - `network` (MicroPython network module)
     - `ubinascii` (MicroPython binascii module)
     - `umqtt` (MicroPython MQTT module)
     - `urequests` (MicroPython requests module)
     - [micropython-stepper](https://pypi.org/project/micropython-stepper/) (Custom Stepper module)
     - [micropython-servo](https://pypi.org/project/micropython-servo/) (Custom Servo module)
     - `mysecrets` (Custom module for storing Wi-Fi and MQTT broker information)

## How to Use

1. Clone the repository to your local machine.
2. Install the required dependencies.
3. Run the main control program `surgical_robot_main.py` on your computer.
4. Flash the MicroPython script `pico_code.py` onto the Raspberry Pi Pico.
5. Power on the Pico to initiate the robot's autonomous operation.

**Note:** Adjust configurations, link lengths, and other parameters as needed for your specific setup.
```
