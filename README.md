# Autonomous Operation Robot - ME-35 Final Project

This repository contains the code for an autonomous operation robot developed as part of the ME-35 (Introduction to Robotics) final project. Inspired by the groundbreaking daVinci surgical robot which took the internet by storm in 2017 when “they did surgery on a grape” in a promotional video, the goal of this project was to combine everything we have learned this semester to create a robot that can autonomously retrieve a game piece from the board game Operation. 

Check out a video of the project [here](https://youtu.be/dTK5n1j3qqo).

## mysimpleik.py

- **Summary:**
  - A custom library providing functions for forward and inverse kinematics calculations, used in the robotic control system.

- **Functions:**
  - `fwdkin(angles=[0,0], L=[14.5, 14.8])`: Calculate forward kinematics to determine end-effector coordinates.
  - `invkin(coords=[12, 12], L=[14.5, 14.8], servo_1_lim=[10, 150], servo_2_lim=[32, 120])`: Calculate inverse kinematics to determine servo angles.

- **Dependencies:**
  - `numpy`

## surgical_robot_main.py

- **Summary:**
  - The main control program that utilizes computer vision to identify game piece coordinates and sends movement commands to the robot through MQTT.

- **Functions:**
  - `mqtt_setup()`: Set up and connect to the MQTT broker.
  - `get_frame(vid)`: Capture and return a frame from the camera and handle errors with continunity camera connection.
  - `process_image(frame, real_length, pixel_length, thresh_area)`: Process the frame to identify game piece coordinates.
  - `check_board(vid, thresh_area)`: Check if a game piece is present on the board.
  - `get_coords(vid, pixel_length, real_length, thresh_area, es=0.1, dt=0.1, i_timeout=20)`: Get coordinates of the identified game piece.
  - `send_data(client, topic, message)`: Publish data to the MQTT broker.
  - `move_stepper(client, pos)`: Move the stepper motor to a specified position.
  - `move_servo(client, init_pos, end_pos, step_size=1, reverse=False)`: Move arm to a specified position.
  - `locate_part()`: Locate the game piece and return its coordinates.
  - `success_indication(client)`: Move servo arm to indicate a successful operation.

- **Dependencies:**
  - `cv2` (OpenCV)
  - `numpy`
  - `mysimpleik` (Custom Inverse Kinematics module)
  - `paho.mqtt.client` (Python3 MQTT module)

## pico_code.py

- **Summary:**
  - MicroPython script designed for the Raspberry Pi Pico microcontroller. It acts as the interface between hardware components and MQTT, receiving commands and controlling the robot's movements.

- **Functions:**
  - `setup()`: Initialize hardware components.
  - `connect_wifi(wifi)`: Connect to a Wi-Fi network.
  - `move_stepper(message)`: Move the stepper motor based on received MQTT messages.
  - `move_base(angle)`: Move the base servo to a specified angle.
  - `move_elbow(angle)`: Move the elbow servo to a specified angle.
  - `control_suction(message)`: Activate/deactivate the suction mechanism based on received MQTT messages.
  - `whenCalled(topic, msg)`: Callback function for processing MQTT messages.
  - `main()`: Main function for handling MQTT communication and controlling the robot.

- **Dependencies:**
  - `umqtt` (MicroPython MQTT module)
  - `stepper` ([micropython-stepper](https://pypi.org/project/micropython-stepper/) module)
  - `servo` ([micropython-servo](https://pypi.org/project/micropython-servo/) module)
  - `mysecrets` (Custom module for storing Wi-Fi and MQTT broker information)
