#!/usr/bin/env python3

import roslib
import rospy
import time
import math
import sys
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Float64


class camera_rotation_controller:
    def __init__(self):
        rospy.init_node('camera_rotation_controller')
        
        # Initialize points for camera and marker centers
        self.cam_center = Point()
        self.current_marker_center = Point()
        self.detected_marker_ids = []
        self.marker_positions = {}  # Stores (x, y) positions for each detected marker
        self.current_id = 0
        self.collecting_info = True  # Flag to manage data collection phase
        self.marker_target_reached = False
        self.active_marker = 0
        self.angle = 0.0
        
        # Publishers
        self.output_image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.vertical_stand_pub = rospy.Publisher('/exp_robot4/vertical_stand_position_controller/command', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/marker/id_number", Int32, self.id_callback, queue_size=1)
        rospy.Subscriber("/marker/center_loc", Point, self.marker_center_callback, queue_size=1)
    
    def camera_info_callback(self, msg):
        # Determine the center of the camera frame
        self.cam_center.x = msg.height / 2
        self.cam_center.y = msg.width / 2
    
    def id_callback(self, msg):
        # Update the current marker ID
        self.current_id = msg.data
    
    def marker_center_callback(self, msg):
        # Update the marker's center position
        self.current_marker_center.x = msg.x
        self.current_marker_center.y = msg.y
        if self.current_id and self.current_id not in self.detected_marker_ids:
            self.marker_positions[self.current_id] = (msg.x, msg.y)
    
    def image_callback(self, msg):
        # Convert the incoming image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        control_msg = Float64()
        
        if self.collecting_info:
            if len(self.detected_marker_ids) < 7:
                self.angle += 0.06
                control_msg.data = self.angle
                if self.current_id and self.current_id not in self.detected_marker_ids:
                    self.detected_marker_ids.append(self.current_id)
                    self.detected_marker_ids.sort()
                    rospy.loginfo(f"Markers detected: {self.detected_marker_ids}")
            else:
                self.collecting_info = False
                rospy.loginfo("All markers collected. Switching to navigation mode.")
        else:
            if self.detected_marker_ids:
                self.active_marker = self.detected_marker_ids[0]
                target_x = self.current_marker_center.x
                target_y = self.current_marker_center.y
                rospy.loginfo(f"Target X: {target_x}, Camera Center X: {self.cam_center.x}, Current Marker ID: {self.active_marker}")
                
                if abs(self.cam_center.x - target_x) < 20 and self.current_id == self.active_marker:
                    self.marker_target_reached = True
                    control_msg.data = self.angle
                    rospy.loginfo(f"Marker {self.active_marker} reached.")
                    
                    # Highlight the marker on the image
                    cv2.circle(image, (int(target_x), int(target_y)), 25, (0, 255, 0), 4)
                    
                    # Publish the annotated image
                    annotated_image = CompressedImage()
                    annotated_image.header.stamp = rospy.Time.now()
                    annotated_image.format = "jpeg"
                    annotated_image.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
                    self.output_image_pub.publish(annotated_image)
                
                if self.marker_target_reached:
                    self.detected_marker_ids.pop(0)
                    self.marker_target_reached = False
                    control_msg.data = self.angle
                elif self.cam_center.x > target_x and self.current_id == self.active_marker:
                    self.angle += 0.01
                    control_msg.data = self.angle
                    rospy.loginfo("Adjusting left.")
                elif self.cam_center.x < target_x and self.current_id == self.active_marker:
                    self.angle -= 0.01
                    control_msg.data = self.angle
                    rospy.loginfo("Adjusting right.")
                else:
                    self.angle += 0.06
                    control_msg.data = self.angle
                    rospy.loginfo("Continuing adjustment.")
            else:
                control_msg.data = self.angle
                rospy.loginfo("All markers have been processed.")
        
        self.vertical_stand_pub.publish(control_msg)


def main():
    controller = camera_rotation_controller()
    rospy.spin()

if __name__ == '__main__':
    main()


