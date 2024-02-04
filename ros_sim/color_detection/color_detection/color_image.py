#!/usr/bin/python3

'''
FUNCTIONS FOR COLOR DETECTION CODE
'''

import cv2 
import numpy as np

# Resize and show image 
def show_image(img, window_name): 
    img_res = cv2.resize(img, None, fx=0.3, fy=0.3)
    cv2.imshow(window_name, img_res)
    cv2.waitKey(1)


# Get color limits
def get_color_range(color):
    
    # Complete only for the color you want to detect 

    if color == 'green':
        # lower_range = np.array([0, 100, 100])  # Lower range for red in HSV
        # upper_range = np.array([10, 255, 255]) 
       
        lower_range = np.array([40, 40, 40])
        upper_range = np.array([80, 255, 255])
 
    #elif color == 'red':
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>

    #elif color == 'blue':
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>

    #else:  # Yellow 
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>
    
    return lower_range, upper_range


# Detects the color 
def detect_color(img, lower_range, upper_range):
    # Convert image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask using the specified color range
    mask = cv2.inRange(hsv, lower_range, upper_range)

    # Define rectangular kernel 25x25
    kernel = np.ones((25, 25), np.uint8)

    # Apply opening to mask
    mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return mask_opened


# Get maximum contour, area and its center 
def get_max_contour(mask): 
    contour_max = []
    area_max = 0
    center = (-1, -1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # For each contour
    for contour in contours:
        # Get area of the contour 
        area = cv2.contourArea(contour)

        # If area is bigger than area_max 
        if area > area_max:
            # Update area max value 
            area_max = area

            # Update contour_max value
            contour_max = contour

            # Get center of the contour using cv2.moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center = (cX, cY)

    return contour_max, area_max, center
