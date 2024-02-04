#!/usr/bin/python3

import cv2 
from geometry_msgs.msg import Twist

# Variables 
########################################################################
max_area_detection = 180000.0  # <COMPLETE: Set the maximum area for too close detection>
min_area_detection = 100000.0   # <COMPLETE: Set the minimum area for too far detection>


# Functions 
########################################################################

# Gets the robot speed and direction depending on the color detection 
def get_velocity(mid_width, x, area):

    vel = Twist()

    #########################################################################################
    # If the area is too big, robot too close --> the robot go backwards                    #
    # If the area is too small, robot far --> the robot go towards the color                #
    # If the area is between min_area_detection and max_area_detection --> the robot stops  # 
    #########################################################################################
    print(area)

    # if too close -> go backwards 8towards
    if area > max_area_detection:
        vel.linear.x = -0.3
        # The robot will turn to the color to keep the detection in the middle of the image 
        vel.angular.z = get_angular_velocity(x, mid_width)

    # if too far -> move towards
    elif area < min_area_detection:
        vel.linear.x = 0.2  # Set the forward linear velocity

        # The robot will turn to the color to keep the detection in the middle of the image 
        vel.angular.z = get_angular_velocity(x, mid_width)  # Call get_angular_velocity function
        print(f"Angular Velocity: {vel.angular.z}, X: {x}, Mid Width: {mid_width}")

    # if good distance --> stop
    else:
        vel.linear.x = 0.0
        vel.angular.z = 0.0

    return vel


# Gets the angular velocity if the color detection is on one side of the image to turn towards the color 
def get_angular_velocity(x, mid_width):
    vel_z = 0.0
    offset = 70  # <COMPLETE: Set the offset value>

    # If the centroid of the color is on the right part of the image + offset
    if x > mid_width + offset:
        # Turn to the right
        vel_z = -0.1  # <COMPLETE: Set the angular velocity for turning right>

    # If the centroid of the color is on the left part of the image - offset
    elif x < mid_width - offset:
        # Turn to the left
        vel_z = 0.1  # <COMPLETE: Set the angular velocity for turning left>

    return vel_z
