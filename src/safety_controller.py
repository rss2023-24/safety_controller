#!/usr/bin/env python2

import numpy as np

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_tools import *

class SafetyController:
    LASER_SCAN_TOPIC = "/scan"
    NAV_OUTPUT_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"
    SAFETY_DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"
    MIN_ANGLE = -2.35500001907
    MAX_ANGLE = 2.35500001907
    ANGLE_INCREMENT = 0.0475757569075 #1081 pieces of data or 0.2498 degrees per point!
    LASER_MIDPOINT = 541


    def __init__(self):
        # Initializes subscribers
        self.laser_data = None
        self.last_drive_command = None
        rospy.Subscriber(self.LASER_SCAN_TOPIC, LaserScan, self.gatherLaserData)
        rospy.Subscriber(self.NAV_OUTPUT_TOPIC, AckermannDriveStamped, self.gatherDriveCommand)

        # Initializes publishers
        self.pub = rospy.Publisher(self.SAFETY_DRIVE_TOPIC, AckermannDriveStamped) 

        # Configurable Parameters
        self.num_items_in_avg = 10 # TODO current value needs to be tuned
        self.num_items_per_side = 20 # Approximately 10 degrees (0.25 degrees per item)

        # Maps distances from obstacle to maximum forward speed allowed
        # Ex: from 0 to 0.2 units away, allow a maximum speed of 0
        # Ex: from 0.2 to 0.4 units away, allow a maximum speed of 0.1
        # Ex: from 0.4 to infinite units away, have no speed limit
        self.range_list = [ # TODO current values are just examples
            (0.2,  0),
            (0.4, 0.1),
            (999, 999)
        ]

    # Gather laser scan data
    def gatherLaserData(self, laser_scan):
        self.laser_data = laser_scan
        self.handleSafety()

    # Gather last drive command
    def gatherDriveCommand(self, last_drive_command):
        self.last_drive_command = last_drive_command
        self.handleSafety()

    # Handles safety controller logic
    def handleSafety(self):
        # Averages closest num_items_in_avg points to estimate distance to nearest obstacle
        laser_data = np.array(self.laser_data.ranges[self.LASER_MIDPOINT - self.num_items_in_range: self.LASER_MIDPOINT + self.num_items_in_range + 1])
        sorted_data = np.sort(laser_data)
        distance_to_obstacle = np.average(sorted_data[0:self.num_items_in_avg])

        # Figures out the maximum speed in which we can approach the obstacle
        speed_limit = 0
        for max_distance, max_speed in self.range_list:
            speed_limit = max_speed
            if distance_to_obstacle < max_distance:
                break 
        
        # If command is too fast, override it
        last_command_speed = self.last_drive_command.drive.speed
        if last_command_speed > speed_limit:
            self.controlRobot(0, 0)
    
    # Closest distance from origin to a line with equation y = mx + b
    def distanceToLine(self, m, b):
        return abs(b)/(math.sqrt(m*m + 1))

    # Sends control commands to the robot
    def controlRobot(self, speed, steering_angle, steering_angle_velocity=0, acceleration=0, jerk=0 ):
        drive_command_stamped = AckermannDriveStamped()
        
        # Builds command header
        drive_header = Header()
        drive_header.stamp = rospy.Time.now() 
        drive_header.frame_id = "base_link"
        drive_command_stamped.header = drive_header

        # Builds drive command
        drive_command = AckermannDrive()
        drive_command.speed = speed
        drive_command.steering_angle = steering_angle
        drive_command.steering_angle_velocity = steering_angle_velocity
        drive_command.acceleration = acceleration
        drive_command.jerk = jerk
        drive_command_stamped.drive = drive_command

        # Publishes message
        self.pub.publish(drive_command_stamped)

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()
