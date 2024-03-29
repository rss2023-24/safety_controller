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

    MIN_ANGLE_DEG = -135
    MAX_ANGLE_DEG = 135
    NUM_RANGES = 1081.
    ANGLE_INCREMENT_DEG = (MAX_ANGLE_DEG - MIN_ANGLE_DEG) / NUM_RANGES #1081 pieces of data or 0.2498 degrees per point!

    SIDE_ANGLE_DEG = 30 # angle to center of right/left cones from 0
    FRONT_MIDPOINT = NUM_RANGES // 2
    RIGHT_MIDPOINT = FRONT_MIDPOINT - (SIDE_ANGLE_DEG // ANGLE_INCREMENT_DEG)
    LEFT_MIDPOINT = FRONT_MIDPOINT + (SIDE_ANGLE_DEG // ANGLE_INCREMENT_DEG)


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

    # Gather laser scan data
    def gatherLaserData(self, laser_scan):
        self.laser_data = laser_scan
        self.handleSafety()

    # Gather last drive command
    def gatherDriveCommand(self, last_drive_command):
        self.last_drive_command = last_drive_command

    # Handles safety controller logic
    def handleSafety(self):
        # Sets up variables
        self.num_items_in_avg = rospy.get_param("safety_controller/num_items_in_avg")
        self.num_items_per_side_front = rospy.get_param("safety_controller/num_items_per_side_front")
        self.num_items_per_side_side = rospy.get_param("safety_controller/num_items_per_side_side")
        MIN_DISTANCE_SIDE = rospy.get_param("safety_controller/desired_distance_side")
        MIN_DISTANCE_FRONT = rospy.get_param("safety_controller/desired_distance_front")

        if self.laser_data is None or self.last_drive_command is None:
            return
        
        # Gathers data for our 3 sections
        laser_data = np.array(self.laser_data.ranges)
        last_command_speed = self.last_drive_command.drive.speed

        cone_vals = {
            'right': (self.RIGHT_MIDPOINT, self.num_items_per_side_side),
            'front': (self.FRONT_MIDPOINT, self.num_items_per_side_front),
            'left': (self.LEFT_MIDPOINT, self.num_items_per_side_side), 
        }

        v_const = 0.33
        if last_command_speed <= 1.5:
            travel_distance = last_command_speed * v_const
        else:
            travel_distance = 0.85
        obstacle_distance = dict()
        for direction in cone_vals.keys():
            midpoint, items_per_side = cone_vals[direction]
            midpoint = int(midpoint)
            range_slice = laser_data[midpoint - items_per_side : midpoint + items_per_side]
            sorted_slice = np.sort(range_slice)
            obstacle_distance[direction] = np.median(sorted_slice[:self.num_items_in_avg])

        #print(obstacle_distance)
        stop_front = travel_distance + MIN_DISTANCE_FRONT >= obstacle_distance['front']
        stop_side = (
            (   travel_distance + MIN_DISTANCE_SIDE >= obstacle_distance['right']
                or travel_distance + MIN_DISTANCE_SIDE >= obstacle_distance['left']
            ) and obstacle_distance['front'] < 0.8)
        
        if stop_front or stop_side:
            self.controlRobot(0, 0)

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
