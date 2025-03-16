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

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone",
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.VELOCITY = 0.5 # "Please keep your desired velocities below 1 (meters/sec). Even though the simulator will behave at higher speeds, your real robot will not." - quoted from lab 4 so probably listen to this
        self.relative_x = 0
        self.relative_y = 0

        self.relative_x_allowed_error_meters = 0.1 * self.parking_distance
        # Starting with +- 10 degrees
        self.relative_y_allowed_error_degrees = 10

        # PD Parameters
        self.Kp = 0.3 # 0.3
        self.Kd = 0.21 # 0.21
        self.prev_pd_error = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        # x is in front and y is to the left
        # camera faces forward so wont be able to see cone if its behind so x >= 0
        # probably spatial overlap also not possible so no x = 0 either
        # higher magnitude of y means more angled from center of camera, so if driving straight, minimize this to always keep cone in camera view
        # if distance threshold, publish a negative velocity which is backing up, but PD controller still active which means you turn as you back up automatically

        ###################################################################################################
        # *** Important *** Probably need to run wall follower's safety controller at same time to avoid crashing #
        ###################################################################################################

        # Assume relative_x can't be negative and so it must be greater than 0 at least

        # First calculate pd metrics cause will be logged to bag file even at desired state

        # Get x distance in meters from parking point
        current_distance_to_dest = self.relative_x - self.parking_distance

        # Get y distance in degrees from parking state
        if self.relative_x == 0:
            raise Exception("ZERO_DIVISION_ERROR: self.relative_x is below 0, but should not be")
        current_angle_from_dest = ( 180 / (2 * np.pi) ) * np.arctan(self.relative_y / self.relative_x) # Arctan defined for all inputs so long as self.relative_x isn't 0, but that would be overlap so shouldn't happen

        # Calculate error for pd
        pd_error = current_angle_from_dest # Proportional to degrees, but degrees or meters shouldnt matter. Just did this for ease of use.

        now_time = self.get_clock().now()
        dt = (now_time - self.prev_time).nanoseconds * 1e-9
        if dt == 0:
            dt = 1e-6
        pd_derivative = (pd_error - self.prev_pd_error) / dt

        # Check if already at desired state already and if not steer
        if np.abs(current_distance_to_dest) <= self.relative_x_allowed_error_meters and np.abs(current_angle_from_dest) <= self.relative_y_allowed_error_degrees:
            speed_cmd = 0
            steering_angle = 0
        elif np.abs(current_distance_to_dest) <= self.relative_x_allowed_error_meters:
            steering_angle = -(self.Kp * pd_error + self.Kd * pd_derivative)
            max_steering = 0.34
            steering_angle = max(-max_steering, min(max_steering, steering_angle))
            speed_cmd = -self.VELOCITY
        else:
            steering_angle = -(self.Kp * pd_error + self.Kd * pd_derivative)
            max_steering = 0.34
            steering_angle = max(-max_steering, min(max_steering, steering_angle))
            speed_cmd = self.VELOCITY * np.sign(current_distance_to_dest) # if in front of parking distance, it will start moving backward

        self.prev_pd_error = pd_error
        self.prev_time = now_time

        drive_cmd.drive.speed = float(speed_cmd)
        drive_cmd.drive.steering_angle = float(steering_angle)

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
        error_msg.x_error = float(self.relative_x - self.parking_distance)
        error_msg.y_error = float(self.relative_y)
        error_msg.distance_error = float(np.sqrt(error_msg.x_error**2 + error_msg.y_error**2))

        #################################

        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
