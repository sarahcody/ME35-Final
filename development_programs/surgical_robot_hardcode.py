import cv2
import numpy as np
import math
import time
from mysimpleik import invkin
import paho.mqtt.client as mqtt
from myprojectsecrets import MyBroker
# calibration parameters
pixel_length = 675  # Length in pixels between the markers
real_length = 20  # Corresponding real-world length in centimeters (20 cm diagonal markers used for testing)
thresh_area = 500

# physical properties
arm_lengths = [15.5,14]

# dimensional adjustments
x_offset = -7
y_offset = 0
z_offset = 16.5-14.8

# set up camera device
capture_device_type = 'AVCaptureDeviceTypeContinuityCamera'

# set up mqtt client
def mqtt_setup():
    global client
    client = mqtt.Client("NotFred")
    client.connect(MyBroker['ip'])
    client.subscribe('stepper')
    client.subscribe('servo')
    client.subscribe('suction')
    print(f'Connected to MQTT broker as {client}')
    return client

def get_frame(vid):
    while True:
        #print('Getting Frame')
        ret, frame = vid.read()
        if not ret:
            print('Failed to get image from the camera', flush=True)
            continue  # Continue looping until a frame is successfully captured
        #print('Got a frame')
        return frame

def process_image(frame, real_length=20, pixel_length=402, thresh_area=500):
    # Rotate the frame by 90 degrees clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    cv2_image = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB)
    r, g, b = cv2.split(cv2_image)
    color = cv2.subtract(b, r)
    blurred = cv2.GaussianBlur(color, (3, 3), 0)
    thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    largest_contours = []

    for c in cnts:
        area = cv2.contourArea(c)

        if area > thresh_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
            else:
                cX, cY = 0, 0

            largest_contours.append([area, cX, cY])

    largest_contours.sort(key=lambda x: x[1])  # Sort contours by x-coordinate

    if len(largest_contours) >= 2:
        centroid1 = (largest_contours[0][1], largest_contours[0][2])
        centroid2 = (largest_contours[1][1], largest_contours[1][2])

        x, y, w, h = cv2.boundingRect(cnts[1])
        cv2.rectangle(rotated_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(rotated_frame, (centroid2[0], centroid2[1]), 7, (255, 0, 0), -1)

        real_x = (centroid2[0] - centroid1[0]) * (real_length / pixel_length)
        real_y = (centroid1[1] - centroid2[1]) * (real_length / pixel_length)  # Flip and adjust for negative y

        cv2.putText(rotated_frame, f"Real: ({real_x:.2f} cm, {real_y:.2f} cm)",
                    (centroid2[0] - 20, centroid2[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        text_pixel = f"Pixel: ({centroid2[0]}, {centroid2[1]})"
        cv2.putText(rotated_frame, text_pixel, (centroid2[0] - 20, centroid2[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.line(rotated_frame, centroid1, centroid2, (0, 255, 0), 2)

        length_pixels = int(math.sqrt((centroid2[0] - centroid1[0]) ** 2 + (centroid2[1] - centroid1[1]) ** 2))
        length_cm = (length_pixels / pixel_length) * real_length
        cv2.putText(rotated_frame, f"Length: {length_pixels} pixels, {length_cm:.2f} cm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        origin = (centroid1[0], centroid1[1])
        cv2.line(rotated_frame, origin, (origin[0] + 50, origin[1]), (0, 0, 0), 2)
        cv2.line(rotated_frame, origin, (origin[0], origin[1] - 50), (0, 0, 0), 2)
        cv2.putText(rotated_frame, "(0, 0)", (origin[0] - 20, origin[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Processed Frame", rotated_frame)
    # cv2.imshow("Thresholded Image", thresh)
    # cv2.imshow("Blurred Image", blurred)

    if len(largest_contours) >= 2:
        real_coords = [real_x, real_y]
        return real_coords

    return None

def check_board(vid, thresh_area=500):
    frame = get_frame(vid)
    # Rotate the frame by 90 degrees clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    cv2_image = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB)
    r, g, b = cv2.split(cv2_image)
    color = cv2.subtract(b, r)
    blurred = cv2.GaussianBlur(color, (3, 3), 0)
    thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    blue_contours = [c for c in cnts if cv2.contourArea(c) > thresh_area]

    return len(blue_contours) == 1

def get_coords(vid, pixel_length, real_length, thresh_area, es=0.1, dt=0.1, i_timeout=20):
    print("{:<10} | {:<15} | {:<15} | {:<15}".format("Iteration", "Absolute Error", "X-coordinate", "Y-coordinate"), flush=True)
    print("------------------------------------------------------------")

    ea = 1  # initialize absolute error term, cv frame distance in cm
    i = 0  # loop counter

    while ea > es:
        i += 1
        frame_0 = get_frame(vid)
        cv_coords_0  = process_image(frame_0, pixel_length=pixel_length, real_length=real_length, thresh_area=thresh_area)
        
        time.sleep(dt)
        frame_1 = get_frame(vid)
        cv_coords_1 = process_image(frame_1, pixel_length=pixel_length, real_length=real_length, thresh_area=thresh_area)
        
        if cv_coords_0 is None or cv_coords_1 is None:
            print(f"Error: Two points not identified {i}. Restarting the loop.", flush=True)
            ea = 1
            i = 0
            continue

        x = (cv_coords_1[0] + cv_coords_0[0]) / 2
        y = (cv_coords_1[1] + cv_coords_0[1]) / 2

        ea_x = np.abs(cv_coords_1[0] - cv_coords_0[0])
        ea_y = np.abs(cv_coords_1[1] - cv_coords_0[1])
        ea = max(ea_x, ea_y)

        print("{:<10} | {:<15.6f} | {:<15.6f} | {:<15.6f}".format(i, ea, x, y), flush=True)

        if i > i_timeout:
            res_coords = None
            break
    
    res_coords = [round(x, 3), round(y)]
    return res_coords

def send_data(client, topic, message):
    client.publish(topic, message)
    print(f'message: {message} sent to topic: {topic}')

def move_servo(start_angle, end_angle, client, step_size=2, servo_num=1):
    delay_factor = 10  # Adjust this factor to dial in total time of motion
    no_load_speed = 0.17
    if servo_num == 1:
        servo_topic = 'base'
    if servo_num == 2:
        servo_topic = 'elbow'

    # Calculate the time per degree based on the servo's speed without load
    time_per_degree = no_load_speed / 60
    
    # Calculate the angular distance
    angular_distance = abs(end_angle - start_angle)
    
    # Calculate the total time required for motion accounting for delay factor whcih includes adjusting for loaded speed
    total_time = angular_distance * time_per_degree * delay_factor

    # Calculate the number of steps based on the step size
    num_steps = int(angular_distance / step_size)
    if num_steps == 0:
        num_steps = 1  # Ensure at least one step

    # Calculate the actual step size based on the updated number of steps
    actual_step_size = (end_angle - start_angle) / num_steps

    # Iterate through intermediate angles and publish to MQTT
    for i in range(1, num_steps + 1):
        intermediate_angle = start_angle + i * actual_step_size
        message = str(intermediate_angle)
        send_data(client, servo_topic, message) # post angle to mqtt broker
        time.sleep(total_time / num_steps) # Sleep to control the speed of the servo movement
    # Move to the final angle
    time.sleep(0.25)
    send_data(client, servo_topic, str(end_angle))

def move_stepper(client, pos):
    sleep_time = 1.2*(np.abs(pos)/4)
    if pos == 0:
        sleep_time = 12
    steps = round(pos*(200/4))
    steps_sleep = [steps, sleep_time]
    send_data(client, 'stepper', str(steps_sleep))
    time.sleep(sleep_time)
    print(f'Stepper moved {steps} steps')
    return steps, pos

def move_vertical(client, z_start, z_end, x_pos, step_size=2):
    # Generate a list of pairs of angles using invkin
    angles_list = []
    for z_value in range(z_start, z_end + 1, step_size):
        angles = invkin([z_value, x_pos])
        angles_list.append(angles)

    # Loop through the list and use move_servo to control the arm
    for angles in angles_list:
        move_servo(angles[0], angles[1], client, step_size=2, servo_num=1)


def program_start():
    global vid
    vid = cv2.VideoCapture(1)
    print("-------------------------STARTING PROGRAM-----------------------------------")
    coords = get_coords(vid, pixel_length, real_length, thresh_area)
    z_e = 0 # adjust based on physical system
    x_e = coords[0]
    coords = [coords[0],coords[1],z_e]
    angles = invkin([x_e, z_e], arm_lengths) # input coords and arm lengths in cm
    # Print coordinates and angles
    print(f'Final End Effector Position: {coords}')
    print(f'Final Arm Angles: {angles}')
    return coords, angles

try:
    result = False
    while result == False:
        coords, angles = program_start()
        # setup connection to mqtt broker
        client = mqtt_setup()
        # move stepper to coordinate identified in image
        move_stepper(client, -1*(52-coords[1]))
        # move servos
        move_servo(90, 135, client, step_size=2, servo_num=1)
        time.sleep(1)
        move_servo(90, 30, client, step_size=2, servo_num=2)
        time.sleep(1)
        send_data(client, 'suction', 'on')
        time.sleep(3)
        move_servo(30, 90, client, step_size=2, servo_num=2)
        time.sleep(0.25)
        move_servo(135, 90, client, step_size=2, servo_num=1)
        time.sleep(1)
        move_stepper(client, 0)
        result = check_board(vid) # see if part has been picked up
        print(result)
        send_data(client, 'suction', 'off')

except KeyboardInterrupt:
    print('Keyboard interrupted!')
finally:
    vid.release()
    send_data(client, 'suction', 'off')
    client.disconnect()
    print("-------------------------PROGRAM FINISHED-----------------------------------")
    pass
