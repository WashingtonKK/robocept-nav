"""
Reactive obstacle avoidance node.

Sits between the navigation planner (or teleop) and the base driver.
Reads LiDAR scans, modulates incoming velocity commands to avoid
collisions, and publishes safe /cmd_vel.

Behavior:
  - If obstacle < min_obstacle_distance in front: full stop + rotate away.
  - If obstacle < slowdown_distance: scale down linear speed proportionally.
  - Otherwise: pass through the upstream command unchanged.

This node is always active as a safety layer. Even during teleop, it
prevents driving into walls.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        # Parameters.
        self.declare_parameter('min_obstacle_distance', 0.35)
        self.declare_parameter('slowdown_distance', 0.8)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('scan_angle_front_deg', 60.0)
        self.declare_parameter('escape_angular_vel', 0.8)

        self.min_dist = self.get_parameter('min_obstacle_distance').value
        self.slow_dist = self.get_parameter('slowdown_distance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.scan_angle = math.radians(
            self.get_parameter('scan_angle_front_deg').value
        )
        self.escape_vel = self.get_parameter('escape_angular_vel').value

        # State.
        self.latest_scan = None
        self.upstream_cmd = Twist()

        # Subscribers.
        self.scan_sub = self.create_subscription(
            LaserScan, '/robocept/lidar/scan',
            self._scan_callback, 10,
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/nav_cmd_vel',
            self._upstream_cmd_callback, 10,
        )

        # Publisher.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Run at 20 Hz.
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Obstacle avoider started.')

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _upstream_cmd_callback(self, msg: Twist):
        self.upstream_cmd = msg

    def _get_min_distance_in_front(self) -> tuple[float, float]:
        """
        Get minimum distance and its angle in the front sector.

        Returns:
            (min_distance, angle_of_min) — distance in meters,
            angle in radians (0 = straight ahead, positive = left).
        """
        scan = self.latest_scan
        if scan is None:
            return float('inf'), 0.0

        min_dist = float('inf')
        min_angle = 0.0
        half_fov = self.scan_angle / 2.0

        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            # Normalize to [-pi, pi].
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_fov:
                if r < min_dist:
                    min_dist = r
                    min_angle = angle

        return min_dist, min_angle

    def _control_loop(self):
        safe_cmd = Twist()
        front_dist, obstacle_angle = self._get_min_distance_in_front()

        linear = self.upstream_cmd.linear.x
        angular = self.upstream_cmd.angular.z

        if front_dist < self.min_dist:
            # Emergency: obstacle too close. Stop and rotate away.
            safe_cmd.linear.x = 0.0
            # Rotate away from the obstacle.
            if obstacle_angle >= 0:
                safe_cmd.angular.z = -self.escape_vel
            else:
                safe_cmd.angular.z = self.escape_vel
            self.get_logger().warn(
                f'Obstacle at {front_dist:.2f}m — stopping, rotating away.',
                throttle_duration_sec=2.0,
            )
        elif front_dist < self.slow_dist and linear > 0.0:
            # Slowdown zone: scale linear speed proportionally.
            scale = (front_dist - self.min_dist) / (
                self.slow_dist - self.min_dist
            )
            scale = max(0.0, min(1.0, scale))
            safe_cmd.linear.x = linear * scale
            safe_cmd.angular.z = angular
        else:
            # Clear: pass through.
            safe_cmd.linear.x = linear
            safe_cmd.angular.z = angular

        # Clamp.
        safe_cmd.linear.x = max(
            -self.max_linear, min(self.max_linear, safe_cmd.linear.x)
        )
        safe_cmd.angular.z = max(
            -self.max_angular, min(self.max_angular, safe_cmd.angular.z)
        )

        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
