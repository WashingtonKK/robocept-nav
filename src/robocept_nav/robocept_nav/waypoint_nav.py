"""
Simple waypoint navigation node.

Drives the robot to a sequence of (x, y, theta) goals using proportional
control on odometry feedback. Publishes to /nav_cmd_vel which is then
filtered through the obstacle_avoider before reaching the base driver.

This is intentionally simple — no costmaps, no path planning, no
recovery behaviors. It's a starting point. For production navigation,
consider using Nav2 (ros-humble-navigation2).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class WaypointNav(Node):

    def __init__(self):
        super().__init__('waypoint_nav')

        # Parameters.
        self.declare_parameter('goal_tolerance_xy', 0.15)
        self.declare_parameter('goal_tolerance_theta', 0.2)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 1.2)

        self.goal_tol_xy = self.get_parameter('goal_tolerance_xy').value
        self.goal_tol_theta = self.get_parameter('goal_tolerance_theta').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value

        # State.
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.goals = []  # list of (x, y, theta)
        self.active = False

        # Subscribers.
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10,
        )

        # Publisher.
        self.cmd_pub = self.create_publisher(Twist, '/nav_cmd_vel', 10)

        # Services.
        self.stop_srv = self.create_service(
            Empty, '/robocept/nav/stop', self._stop_callback,
        )

        # Control loop at 10 Hz.
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            'Waypoint nav started. Publish goals to /robocept/nav/goal '
            'or call /robocept/nav/stop to halt.'
        )

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Extract yaw from quaternion.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def add_goal(self, x: float, y: float, theta: float = 0.0):
        """Add a waypoint goal programmatically."""
        self.goals.append((x, y, theta))
        if not self.active:
            self.active = True
            self.get_logger().info(
                f'Navigating to ({x:.2f}, {y:.2f}, {theta:.2f})'
            )

    def _stop_callback(self, request, response):
        self.goals.clear()
        self.active = False
        # Publish zero velocity.
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Navigation stopped.')
        return response

    def _control_loop(self):
        cmd = Twist()

        if not self.active or not self.goals:
            self.cmd_pub.publish(cmd)
            return

        gx, gy, g_theta = self.goals[0]

        # Distance and angle to goal.
        dx = gx - self.current_x
        dy = gy - self.current_y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = self._normalize_angle(
            angle_to_goal - self.current_theta
        )

        if distance < self.goal_tol_xy:
            # At position — rotate to final heading.
            theta_error = self._normalize_angle(
                g_theta - self.current_theta
            )
            if abs(theta_error) < self.goal_tol_theta:
                # Goal reached.
                self.get_logger().info(
                    f'Reached waypoint ({gx:.2f}, {gy:.2f})'
                )
                self.goals.pop(0)
                if not self.goals:
                    self.active = False
                    self.get_logger().info('All waypoints reached.')
                else:
                    nx, ny, nt = self.goals[0]
                    self.get_logger().info(
                        f'Next waypoint: ({nx:.2f}, {ny:.2f}, {nt:.2f})'
                    )
            else:
                cmd.angular.z = max(
                    -self.max_ang,
                    min(self.max_ang, self.kp_ang * theta_error),
                )
        elif abs(heading_error) > 0.5:
            # Turn toward goal first.
            cmd.angular.z = max(
                -self.max_ang,
                min(self.max_ang, self.kp_ang * heading_error),
            )
        else:
            # Drive toward goal.
            cmd.linear.x = max(
                -self.max_lin,
                min(self.max_lin, self.kp_lin * distance),
            )
            cmd.angular.z = max(
                -self.max_ang,
                min(self.max_ang, self.kp_ang * heading_error),
            )

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
