#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from pyzbar.pyzbar import decode

import cv2
import math
import numpy as np


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        # Convert ROS camera images to OpenCV images
        self.bridge = CvBridge()

        # -------------------------------------------------
        # STATE VARIABLES
        # -------------------------------------------------

        # Stores:
        # "left"
        # "right"
        # "stop"
        self.qr_command = None

        # Possible states:
        # FORWARD
        # TURNING
        # CLEARING
        # STOPPED
        self.state = "FORWARD"

        # Distance in front of robot
        self.front_distance = 10.0

        # Robot's current angle
        self.current_yaw = 0.0

        # Angle robot should reach when turning
        self.target_yaw = 0.0

        # Used after completing a turn
        self.clear_count = 0

        # Number of control-loop cycles to move forward
        # after completing a turn
        # 15 × 0.1 sec = 1.5 seconds
        self.clear_cycles = 15

        # -------------------------------------------------
        # SUBSCRIBERS
        # -------------------------------------------------

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

        # -------------------------------------------------
        # VELOCITY PUBLISHER
        # -------------------------------------------------

        self.vel_pub = self.create_publisher(
            Twist,
            '/wheel_controller/cmd_vel_unstamped',
            10
        )

        # Control loop runs every 0.1 seconds
        self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Maze Solver Started!")


    # =====================================================
    # CAMERA
    # =====================================================

    def camera_callback(self, img):

        # Convert ROS image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(
            img,
            desired_encoding='bgr8'
        )

        # Decode QR codes using pyzbar
        decoded_objects = decode(frame)

        for obj in decoded_objects:

            # Get QR text
            data = obj.data.decode('utf-8').lower().strip()

            # Draw rectangle around QR code
            x, y, w, h = obj.rect

            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            # Display QR command on camera window
            cv2.putText(
                frame,
                f'QR: {data}',
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

            # Only accept a new command while driving forward
            if (
                self.state == "FORWARD"
                and self.qr_command is None
                and data in ["left", "right", "stop"]
            ):
                self.qr_command = data

                self.get_logger().info(
                    f"QR Detected: {data}"
                )

        cv2.imshow(
            'QR Detection',
            frame
        )

        cv2.waitKey(1)


    # =====================================================
    # LIDAR
    # =====================================================

    def lidar_callback(self, msg):

        ranges = np.array(
            msg.ranges,
            dtype=np.float32
        )

        # Take rays directly in front of robot
        front_ranges = np.concatenate([
            ranges[-20:],
            ranges[:20]
        ])

        # Remove inf and nan values
        valid_ranges = front_ranges[
            np.isfinite(front_ranges)
        ]

        if len(valid_ranges) > 0:

            self.front_distance = float(
                np.mean(valid_ranges)
            )


    # =====================================================
    # IMU
    # =====================================================

    def imu_callback(self, msg):

        # Quaternion from IMU
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Quaternion → yaw angle
        self.current_yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )


    # =====================================================
    # MAIN CONTROL LOOP
    # =====================================================

    def control_loop(self):

        vel = Twist()

        # -------------------------------------------------
        # FORWARD
        # -------------------------------------------------

        if self.state == "FORWARD":

            vel.linear.x = 0.15
            vel.angular.z = 0.0

            # Wall / obstacle reached
            if self.front_distance <= 0.45:

                # STOP QR
                if self.qr_command == "stop":

                    vel.linear.x = 0.0
                    vel.angular.z = 0.0

                    self.state = "STOPPED"

                    self.get_logger().info(
                        "STOP command - Stopping!"
                    )

                # LEFT or RIGHT QR
                elif self.qr_command in ["left", "right"]:

                    # Stop forward movement first
                    vel.linear.x = 0.0

                    # Calculate desired angle
                    self.target_yaw = (
                        self.calculate_target_yaw()
                    )

                    self.state = "TURNING"

                    self.get_logger().info(
                        f"Turning {self.qr_command}..."
                    )

                # Wall detected but no QR instruction
                else:

                    vel.linear.x = 0.0
                    vel.angular.z = 0.0


        # -------------------------------------------------
        # TURNING
        # -------------------------------------------------

        elif self.state == "TURNING":

            vel.linear.x = 0.0

            # Calculate difference between target and
            # current robot angle
            yaw_diff = self.normalize_angle(
                self.target_yaw
                - self.current_yaw
            )

            if self.qr_command == "left":

                vel.angular.z = 0.3

            elif self.qr_command == "right":

                vel.angular.z = -0.3

            # Is the robot close enough to target?
            if abs(yaw_diff) < 0.1:

                vel.linear.x = 0.0
                vel.angular.z = 0.0

                self.get_logger().info(
                    "Turn complete"
                )

                # Forget old QR instruction
                self.qr_command = None

                # Move away from intersection
                self.clear_count = 0
                self.state = "CLEARING"


        # -------------------------------------------------
        # CLEARING
        # -------------------------------------------------

        elif self.state == "CLEARING":

            # Drive straight for a short time after turn
            vel.linear.x = 0.15
            vel.angular.z = 0.0

            self.clear_count += 1

            if self.clear_count >= self.clear_cycles:

                self.state = "FORWARD"

                self.get_logger().info(
                    "Intersection cleared - moving forward"
                )


        # -------------------------------------------------
        # STOPPED
        # -------------------------------------------------

        elif self.state == "STOPPED":

            vel.linear.x = 0.0
            vel.angular.z = 0.0


        # Send velocity command to wheel controller
        self.vel_pub.publish(vel)


    # =====================================================
    # CALCULATE TARGET TURN ANGLE
    # =====================================================

    def calculate_target_yaw(self):

        # LEFT = +90 degrees
        if self.qr_command == "left":

            return self.normalize_angle(
                self.current_yaw + math.pi / 2
            )

        # RIGHT = -90 degrees
        elif self.qr_command == "right":

            return self.normalize_angle(
                self.current_yaw - math.pi / 2
            )

        return self.current_yaw


    # =====================================================
    # KEEP ANGLE BETWEEN -PI AND +PI
    # =====================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MazeSolver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Stop robot before shutting down
    stop_msg = Twist()
    node.vel_pub.publish(stop_msg)

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()