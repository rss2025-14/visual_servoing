#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "/drive")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        # Used for Pure Pursuit
        self.look_ahead = 0.5 # Lookahead distance
        self.wheelbase = 0.34 # Length of wheelbase

        # Used for realignment
        self.mb = False # Boolean used to reverse when robot is close to cone but not facing it
        self.max_steer_angle = 0.34
        self.backward_count = 0

        self.get_logger().info("Parking Controller Initialized")

    def find_ref_point(self, cx, cy):
        """
        Compute a reference point at `look_ahead` distance in the direction of the cone.
        If the cone is closer than that, the reference point is the cone itself.
        """
        la_dist = self.look_ahead
        cone_pos = np.array([cx, cy])
        dist = np.sqrt(cx**2 + cy**2)
        if dist <= la_dist:
            return cone_pos, dist
        else:
            return ((cone_pos / dist) * la_dist, dist)

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        L = self.wheelbase
        L1 = self.look_ahead

        # Find the next reference point to pursue and calculate distance error and angle of reference point
        ref, c_dist = self.find_ref_point(self.relative_x, self.relative_y) 
        rx, ry = ref
        dist_err = c_dist - self.parking_distance
        eta = np.arctan2(ry, rx) 

        # If the robot is outside of 0.01 meters from the cone, move towards the cone with pure pursuit
        # Else if the robot is within 0.01 meters from the cone, stop and align with the cone
        if self.mb is False:
            if dist_err > 0.01:
                speed= 1.0
                steer_angle = np.arctan2(2*np.sin(eta)*L, L1)  
            else:
                cone_angle = np.arctan2(self.relative_y, self.relative_x)
                if np.abs(cone_angle) > np.radians(10): 
                    self.mb = True
                speed= 0.0
                steer_angle = 0.0
        else:
            pass #TODO Implement realignment


        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.steering_angle = steer_angle
        drive_cmd.drive.speed = speed

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        x, y = self.relative_x, self.relative_y
        error_msg.x_error = x
        error_msg.y_error = y
        error_msg.distance_error = np.sqrt(x**2 + y**2)

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()