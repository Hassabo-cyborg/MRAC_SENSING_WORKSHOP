#!/usr/bin/python3

''' 
Code to detect and follow a color
    
Run the color_detection launch commands before this file

Execute with python3 color_detection.py 

Complete this template and the template files color_image and velocity
'''

# Import libraries
########################################################################
import cv2
import numpy as np 
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from color_image import show_image, get_color_range, detect_color, get_max_contour
from velocity import get_velocity


# Variables 
########################################################################
bridge = CvBridge()
min_detection = 500  # <COMPLETE: Set the minimum area for color detection>


# Image callback -> it is called when the topic receives information
################################################################
def image_callback(msg):
    
    rospy.loginfo("Image received")

    # Get image and publisher 
    ##########################################

    # Convert your ros image message to opencv using bridge
    img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Show image using show_image function
    show_image(img, "window !")

    # Get half width of the image 
    mid_width = img.shape[1] // 2

    # Create velocity publisher and variable of velocity
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel = Twist()
    
    # Do color detection 
    ##########################################

    # Get color range using get_color_range from color_image.py
    color_range = get_color_range('green')  # <COMPLETE: Call get_color_range function>

    # Get color mask using detect_color from color_image.py
    lower_range, upper_range = color_range
    color_mask = detect_color(img, lower_range, upper_range)
                                                            # <COMPLETE: Call detect_color function>

    # Show mask 
    show_image(color_mask, "Color Mask")

    # Find contours and get max area using get_max_contours from color_image.py 
    contour_max, area_max, center = get_max_contour(color_mask)  # <COMPLETE: Call get_max_contour function>
    print("Maximum area: ", area_max)


    # Get robot speed     
    ##########################################

    # If the area of the detected color is big enough
    if area_max > min_detection:
        print("Cylinder detected")

        # Draw contour and center of the detection and show image 
        cv2.drawContours(img, [contour_max], -1, (0, 255, 0), 2)
        M = cv2.moments(contour_max)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(img, f"Center: ({cX}, {cY})", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Show the image with contour and center
            show_image(img, "Detected Color !")

            # Gets the color speed and direction depending on the color detection using get_velocity from velocity.py
            vel = get_velocity(mid_width, cX, area_max)  # <COMPLETE: Call get_velocity function>

    # If the area of the detected color is not big enough, the robot spins 
    else:
        print("Looking for color: spinning")
        vel.angular.z = 0.2  # <COMPLETE: Set the angular velocity for spinning>

    # Publish velocity
    pub.publish(vel)


# Init node and subscribe to image topic 
################################################################
def main():
    # Init node 'color_detection'
    rospy.init_node("color_detection")

    # Subscribe to image topic and add callback + spin
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
