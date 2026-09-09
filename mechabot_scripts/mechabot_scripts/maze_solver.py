#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import math
import numpy as np

from pyzbar.pyzbar import decode


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        # Bridge
        self.bridge = CvBridge()

        # State variables
        self.qr_command = None
        self.state = "FORWARD"

        self.front_distance = 10.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0

        # Subscriptions
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu/out',
            self.imu_callback,
            10
        )

        # Publisher
        self.vel_pub = self.create_publisher(
            Twist,
            '/wheel_controller/cmd_vel_unstamped',
            10
        )

        # Control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Maze Solver Started!")


    def camera_callback(self, img):

        frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')

        # Decode QR codes using pyzbar
        qr_codes = decode(frame)

        for qr in qr_codes:

            # Read QR data
            data = qr.data.decode('utf-8').lower().strip()

            # Get QR rectangle
            x, y, w, h = qr.rect

            # Draw bounding box
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            # Store QR command
            if data in ["left", "right", "stop"]:

                if self.qr_command is None:

                    self.qr_command = data

                    self.get_logger().info(
                        f"QR Detected: {data}"
                    )

            # Display QR data
            cv2.putText(
                frame,
                f'QR: {data}',
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.imshow('QR Detection', frame)
        cv2.waitKey(1)


    def lidar_callback(self, msg):

        # Get front distance
        ranges = np.array(msg.ranges)

        self.front_distance = np.concatenate(
            [ranges[-20:], ranges[:20]]
        ).mean()


    def imu_callback(self, msg):

        # Quaternion
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert quaternion to yaw
        self.current_yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )


    def control_loop(self):

        vel = Twist()

        if self.state == "FORWARD":

            # Move forward
            vel.linear.x = 0.15
            vel.angular.z = 0.0

            # Obstacle ahead
            if self.front_distance <= 0.45:

                if self.qr_command == "stop":

                    self.state = "STOPPED"

                    self.get_logger().info(
                        "STOP command - Stopping!"
                    )

                elif self.qr_command in ["left", "right"]:

                    self.state = "TURNING"

                    self.target_yaw = self.calculate_target_yaw()

                    self.get_logger().info(
                        f"Turning {self.qr_command}..."
                    )

                else:

                    # Stop if no QR command
                    vel.linear.x = 0.0


        elif self.state == "TURNING":

            # Stop forward movement
            vel.linear.x = 0.0

            if self.qr_command == "left":

                vel.angular.z = 0.3

            elif self.qr_command == "right":

                vel.angular.z = -0.3


            # Check turn completion
            yaw_diff = self.normalize_angle(
                self.target_yaw - self.current_yaw
            )

            if abs(yaw_diff) < 0.1:

                self.state = "FORWARD"

                self.qr_command = None

                self.get_logger().info(
                    "Turn complete, moving forward"
                )


        elif self.state == "STOPPED":

            vel.linear.x = 0.0
            vel.angular.z = 0.0


        # Publish velocity
        self.vel_pub.publish(vel)


    def calculate_target_yaw(self):

        if self.qr_command == "left":

            return self.normalize_angle(
                self.current_yaw + 1.57
            )

        elif self.qr_command == "right":

            return self.normalize_angle(
                self.current_yaw - 1.57
            )

        return self.current_yaw


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


def main(args=None):

    rclpy.init(args=args)

    node = MazeSolver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()