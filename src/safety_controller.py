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

class SafetyController:
    LASER_SCAN_TOPIC = "/scan"
    NAV_OUTPUT_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"
    SAFETY_DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"
    MIN_ANGLE = -2.35500001907
    MAX_ANGLE = 2.35500001907
    ANGLE_INCREMENT = 0.0475757569075 #1081 pieces of data or 0.2498 degrees per point!
    LASER_MIDPOINT = 541
    # MIN_DISTANCE = rospy.get_param("safety_controller/desired_distance")
    # TIME_CONST = rospy.get_param("safety_controller/time_constant")


    def __init__(self):
        # Initializes subscribers
        self.laser_data = None
        self.last_drive_command = None
        rospy.Subscriber(self.LASER_SCAN_TOPIC, LaserScan, self.gatherLaserData)
        rospy.Subscriber(self.NAV_OUTPUT_TOPIC, AckermannDriveStamped, self.gatherDriveCommand)

        # Initializes publishers
        self.pub = rospy.Publisher(self.SAFETY_DRIVE_TOPIC, AckermannDriveStamped) 

        # Configurable Parameters
        self.num_items_in_avg = 0
        self.num_items_per_side = 0
        # self.num_items_in_avg = rospy.get_param("safety_controller/num_items_in_avg") # TODO current value needs to be tuned
        # self.num_items_per_side = rospy.get_param("safety_controller/num_items_per_side") # Approximately 10 degrees (0.25 degrees per item)

    # Gather laser scan data
    def gatherLaserData(self, laser_scan):
        self.laser_data = laser_scan
        self.handleSafety()

    # Gather last drive command
    def gatherDriveCommand(self, last_drive_command):
        self.last_drive_command = last_drive_command

    # Handles safety controller logic
    def handleSafety(self):
        self.num_items_in_avg = rospy.get_param("safety_controller/num_items_in_avg") # TODO current value needs to be tuned
        self.num_items_per_side = rospy.get_param("safety_controller/num_items_per_side")
        MIN_DISTANCE = rospy.get_param("safety_controller/desired_distance")
        TIME_CONST = rospy.get_param("safety_controller/time_constant")
        if self.laser_data is None or self.last_drive_command is None:
            return
        # Averages closest num_items_in_avg points to estimate distance to nearest obstacle
        slice_start = self.LASER_MIDPOINT - self.num_items_per_side
        slice_end = slice_start + (2 * self.num_items_per_side)
        laser_data = np.array(self.laser_data.ranges)[slice_start:slice_end]
        sorted_data = np.sort(laser_data)
        distance_to_obstacle = np.average(sorted_data[0:self.num_items_in_avg])

        last_command_speed = self.last_drive_command.drive.speed
        print('speed*time_const', last_command_speed * TIME_CONST)
        if (last_command_speed * TIME_CONST) + MIN_DISTANCE >= distance_to_obstacle:
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
