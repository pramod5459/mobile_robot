import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math


class LidarObstacle(Node):

    def __init__(self):
        super().__init__("lidar_obstacle")

        # Subscribe to LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10
        )

        # Publish velocity commands
        self.publisher = self.create_publisher(
            Twist,
            "/wheel_controller/cmd_vel_unstamped",
            10
        )

        # -----------------------------
        # DISTANCE THRESHOLDS
        # -----------------------------

        # If obstacle is closer than this in FRONT,
        # stop forward motion and turn.
        self.front_danger = 0.45

        # Only correct for a side obstacle if it is
        # REALLY close.
        self.side_danger = 0.25

        # -----------------------------
        # SPEEDS
        # -----------------------------

        self.forward_speed = 0.20

        # Slower while correcting near a wall
        self.correction_speed = 0.15

        # Strong turn when front is blocked
        self.turn_speed = 0.80

        # Gentle steering correction
        self.correction_turn = 0.25


    def get_min_distance(self, ranges):

        valid_ranges = []

        for distance in ranges:

            if math.isfinite(distance) and distance > 0.0:
                valid_ranges.append(distance)

        if len(valid_ranges) == 0:
            return float("inf")

        return min(valid_ranges)


    def lidar_callback(self, msg):

        # ------------------------------------------------
        # LiDAR regions
        #
        # Keep these sectors if they were working correctly
        # with your current LiDAR orientation.
        # ------------------------------------------------

        range_right = msg.ranges[20:140]
        range_front = msg.ranges[140:220]
        range_left = msg.ranges[220:340]

        right = self.get_min_distance(range_right)
        front = self.get_min_distance(range_front)
        left = self.get_min_distance(range_left)

        print(
            f"LEFT: {left:.2f} | "
            f"FRONT: {front:.2f} | "
            f"RIGHT: {right:.2f}"
        )

        cmd = Twist()

        # ==================================================
        # PRIORITY 1:
        # OBSTACLE DIRECTLY IN FRONT
        # ==================================================

        if front < self.front_danger:

            cmd.linear.x = 0.0

            # Turn toward whichever side has more space
            if left > right:

                cmd.angular.z = self.turn_speed

                print("FRONT BLOCKED -> TURN LEFT")

            else:

                cmd.angular.z = -self.turn_speed

                print("FRONT BLOCKED -> TURN RIGHT")


        # ==================================================
        # PRIORITY 2:
        # TOO CLOSE TO LEFT SIDE
        #
        # DON'T stop and rotate.
        # Keep moving while steering away.
        # ==================================================

        elif left < self.side_danger:

            cmd.linear.x = self.correction_speed
            cmd.angular.z = -self.correction_turn

            print("LEFT VERY CLOSE -> STEER RIGHT")


        # ==================================================
        # PRIORITY 3:
        # TOO CLOSE TO RIGHT SIDE
        # ==================================================

        elif right < self.side_danger:

            cmd.linear.x = self.correction_speed
            cmd.angular.z = self.correction_turn

            print("RIGHT VERY CLOSE -> STEER LEFT")


        # ==================================================
        # PRIORITY 4:
        # PATH CLEAR
        # ==================================================

        else:

            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

            print("PATH CLEAR -> FORWARD")


        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = LidarObstacle()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        # Send stop command BEFORE shutting ROS down
        stop_msg = Twist()

        try:
            node.publisher.publish(stop_msg)
        except Exception:
            pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()