import cv2
import numpy as np
import math

# calibration parameters
pixel_length = 402  # Length in pixels between the markers
real_length = 20  # Corresponding real-world length in centimeters (20 cm diagonal markers used for testing)
thresh_area = 500

def process_image(frame):
    # Rotate the frame by 90 degrees clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    cv2_image = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB)
    r, g, b = cv2.split(cv2_image)
    color = cv2.subtract(b, r)
    blurred = cv2.GaussianBlur(color, (3, 3), 0)
    thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]
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
    #print(largest_contours)
    largest_contours.sort(key=lambda x: x[1])  # Sort contours by x-coordinate
    if len(largest_contours) >= 2:
        centroid1 = (largest_contours[0][1], largest_contours[0][2])
        centroid2 = (largest_contours[1][1], largest_contours[1][2])

        # Draw a bounding box around the second contour
        x, y, w, h = cv2.boundingRect(cnts[1])
        cv2.rectangle(rotated_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Draw the center point and coordinates of the second contour
        cv2.circle(rotated_frame, (centroid2[0], centroid2[1]), 7, (255, 0, 0), -1)
        
        # Convert pixel coordinates to real-world coordinates
        real_x = (centroid2[0] - centroid1[0]) * (real_length / pixel_length)
        real_y = (centroid1[1] - centroid2[1]) * (real_length / pixel_length)  # Flip and adjust for negative y
        cv2.putText(rotated_frame, f"Real: ({real_x:.2f} cm, {real_y:.2f} cm)",
                    (centroid2[0] - 20, centroid2[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display pixel coordinates
        text_pixel = f"Pixel: ({centroid2[0]}, {centroid2[1]})"
        cv2.putText(rotated_frame, text_pixel, (centroid2[0] - 20, centroid2[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Draw a line connecting centroids
        cv2.line(rotated_frame, centroid1, centroid2, (0, 255, 0), 2)

        # Update length calculation relative to the coordinate system
        length_pixels = int(math.sqrt((centroid2[0] - centroid1[0]) ** 2 + (centroid2[1] - centroid1[1]) ** 2))
        length_cm = (length_pixels / pixel_length) * real_length
        cv2.putText(rotated_frame, f"Length: {length_pixels} pixels, {length_cm:.2f} cm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Draw coordinate plane with flipped vertical axis
        origin = (centroid1[0], centroid1[1])
        cv2.line(rotated_frame, origin, (origin[0] + 50, origin[1]), (0, 0, 0), 2)
        cv2.line(rotated_frame, origin, (origin[0], origin[1] - 50), (0, 0, 0), 2)
        cv2.putText(rotated_frame, "(0, 0)", (origin[0] - 20, origin[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Processed Frame", rotated_frame)
    cv2.imshow("Thresholded Image", thresh) # show threshold image
    cv2.imshow("Blurred Image", blurred)

# set up camera device
capture_device_type = 'AVCaptureDeviceTypeContinuityCamera'
vid = cv2.VideoCapture(1)

try:
    while True:
        ret, frame = vid.read()
        if not ret:
            break
        process_image(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print('keyboard interrupted!')
finally:
    vid.release()
    cv2.destroyAllWindows()
