#!/usr/bin/python3

import cv2 

# Variables 
########################################################################
max_area_detection = 1000  # <COMPLETE: Set the maximum area for too close detection>
min_area_detection = 100   # <COMPLETE: Set the minimum area for too far detection>


# Functions 
########################################################################

# Gets the robot speed and direction depending on the color detection 
def get_velocity(mid_width, x, area):
    vel = 0.0

    #########################################################################################
    # If the area is too big, robot too close --> the robot go backwards                    #
    # If the area is too small, robot far --> the robot go towards the color                #
    # If the area is between min_area_detection and max_area_detection --> the robot stops  # 
    #########################################################################################
    print(area)
    
    # if too close -> go backwards
    if area > max_area_detection:
        vel = -0.2  # <COMPLETE: Set the backward velocity>

    # if too far -> move towards
    elif area < min_area_detection:
        vel = 0.2  # <COMPLETE: Set the forward velocity>

        # The robot will turn to the color to keep the detection in the middle of the image 
        vel_z = get_angular_velocity(x, mid_width)  # <COMPLETE: Call get_angular_velocity function>
    
        # Add the angular velocity to the linear velocity
        vel += vel_z

    # if good distance --> stop
    else:
        vel = 0.0

    return vel


# Gets the angular velocity if the color detection is on one side of the image to turn towards the color 
def get_angular_velocity(x, mid_width):
    vel_z = 0.0
    offset = 20  # <COMPLETE: Set the offset value>

    # If the centroid of the color is on the right part of the image + offset
    if x > mid_width + offset:
        # Turn to the right
        vel_z = -0.2  # <COMPLETE: Set the angular velocity for turning right>

    # If the centroid of the color is on the left part of the image - offset
    elif x < mid_width - offset:
        # Turn to the left
        vel_z = 0.2  # <COMPLETE: Set the angular velocity for turning left>

    return vel_z
