#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        cpx = ConeLocationPixel()

        if self.LineFollower:
            cv2.rectangle(image, (0, 0), (640, 180), (255, 255, 255), -1) # Only show bottom 50%-75% of image
            cv2.rectangle(image, (0, 270), (640, 360), (255, 255, 255), -1)

        # Find orange object in the image
        bbox = cd_color_segmentation(image)
        bottom_v = bbox[1][1]
        bottom_center_u = (bbox[0][0] + bbox[1][0]) // 2
        cpx.u, cpx.v = float(bottom_center_u), float(bottom_v)
        self.cone_pub.publish(cpx)

        # Publish the image with the bounding box drawn on it
        bbox_top_left = bbox[0][0], bbox[0][1]
        bbox_bottom_right = bbox[1][0], bbox[1][1]
        cv2.rectangle(image, bbox_top_left, bbox_bottom_right, (0, 255, 0), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
