import cv2
import numpy as np
import math
import time
from mysimpleik import invkin
import paho.mqtt.client as mqtt
from myprojectsecrets import MyBroker
# calibration parameters
pixel_length = 720  # Length in pixels between the markers
real_length = 20  # Corresponding real-world length in centimeters (20 cm diagonal markers used for testing)
thresh_area = 500

# physical properties
arm_lengths = [14.5,14.8]

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
    thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)[1]
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
    time.sleep(0.25)

def move_stepper(client, pos):
    if pos < -30:
        pos = -30
    sleep_time = 1.2*(np.abs(pos)/4)
    if pos == 0:
        sleep_time = 12
    steps = round(pos*(200/4))
    steps_sleep = [steps, sleep_time]
    send_data(client, 'stepper', str(steps_sleep))
    time.sleep(sleep_time)
    print(f'Stepper moved {steps} steps')
    return steps, pos

def move_servo(client, init_pos, end_pos, step_size=1, reverse=False):
    # End position
    x_e, z_e = end_pos
    x_0, z_0 = init_pos

    x_vals = np.linspace(x_0, x_e, int(np.abs(x_e - x_0) / step_size) + 1)
    zxs = z_0 * np.ones_like(x_vals)
    z_vals = np.linspace(z_0, z_e, int(np.abs(z_e - z_0) / step_size) + 1)
    xzs = x_e * np.ones_like(z_vals)

    path = np.vstack([np.concatenate([x_vals, xzs]), np.concatenate([zxs, z_vals])])
    print(path)

    angles = np.zeros_like(path)  # Initialize an array to store the angles
    for i in range(len(path[0])):
        x, z = path[:, i]  # Extract coordinates from the current column of path
        current_angles = invkin([x, z])  # Call invkin with the coordinates
        angles[:, i] = current_angles  # Store the angles in the array

    if reverse:
        angles = angles[:, ::-1]  # Reverse the order of angles

    for i in range(len(angles[0])):
        base_angle, elbow_angle = angles[:, i]  # Extract angles from the current column
        send_data(client, "elbow", str(elbow_angle))  
        print(f'move elbow to {elbow_angle}')
        send_data(client, "base", str(base_angle)) 
        print(f'move base to {base_angle}')
        


def locate_part():
    global vid
    vid = cv2.VideoCapture(1)
    
    coords = get_coords(vid, pixel_length, real_length, thresh_area)
    z_e = 1 # adjust based on physical system
    x_e = coords[0]
    coords = [coords[0],coords[1],z_e]
    # Print coordinates and angles
    print(f'Final End Effector Position: {coords}')
    
    return coords

def success_indication(client):
    for i in range(3):
        send_data(client,'elbow', '110')
        time.sleep(0.25)
        send_data(client,'elbow', '70')
        time.sleep(0.25)
    print('retrieved part')

try:
    result = False
    print("-------------------------STARTING PROGRAM-----------------------------------")
    client = mqtt_setup()
    
    # # This code was used for testing inverse kinematics over MQTT with pico
    # angles = invkin([10,1])
    # send_data(client,'base',str(angles[0]))
    # send_data(client,'elbow',str(angles[1]))

    while result == False:
        coords = locate_part()
        print(coords)
        x_final, y_final, z_final = coords
        x_final = x_final-10.5
        y_final = y_final+2
        
        # move stepper to coordinate identified in image
        move_stepper(client, -1*(52-y_final))
        # move servo to suction location
        move_servo(client,[14.8,14.5], [x_final, z_final], step_size=2)
        #send_data(client,'suction','on')
        time.sleep(1) # establish suction
        # reverse the movement to the part
        move_servo(client,[14.8,14.5], [x_final, z_final], step_size=1,reverse=True)
        move_stepper(client,0)
        #send_data(client,'suction','off')

        result = check_board(vid) # see if part has been picked up
        print(result)
    success_indication(client)

except KeyboardInterrupt:
    print('Keyboard interrupted!')
finally:
    #vid.release()
    send_data(client, 'suction', 'off')
    client.disconnect()
    print("-------------------------PROGRAM FINISHED-----------------------------------")
    pass
