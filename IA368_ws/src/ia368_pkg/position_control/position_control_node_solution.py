#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R



class SiegwartController(Node):
    def __init__(self):
        super().__init__("siegwart_position_controller")

        # Parameters (declared so they can be set via launch or CLI)
        self.declare_parameter("Krho", 0.5)
        self.declare_parameter("Kalpha", 1.5)
        self.declare_parameter("Kbeta", -0.6)
        self.declare_parameter("constantSpeed", 0.2)
        self.declare_parameter("backwardAllowed", False)

        self.Krho = self.get_parameter("Krho").value
        self.Kalpha = self.get_parameter("Kalpha").value
        self.Kbeta = self.get_parameter("Kbeta").value
        self.constantSpeed = self.get_parameter("constantSpeed").value
        self.backwardAllowed = self.get_parameter("backwardAllowed").value
        self.get_logger().info('Position control node started.')
        # State
        self.robot_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.goal_received = False

        # Subscribers
        self.create_subscription(Odometry, "myRobot/odom", self.odom_callback, 10)
        self.create_subscription(Pose2D, "myRobot/goal", self.goal_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "myRobot/cmd_vel", 10)

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        # Extract robot pose (x,y,theta) from Odometry
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y

        # Convert quaternion -> yaw
        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])  # x, y, z, w order
        euler = r.as_euler('xyz')               # returns [roll, pitch, yaw] in radians
        yaw = euler[2]
        self.robot_pose.theta = yaw

    def goal_callback(self, msg: Pose2D):
        self.goal_pose = msg
        self.goal_received = True

    def control_law(self):
        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y
        rho = math.hypot(dx, dy)  # distance to goal
        theta_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(theta_goal - self.robot_pose.theta)
        beta = self.normalize_angle(-self.robot_pose.theta - alpha)

        twist = Twist()

        if not self.backwardAllowed:
            # Standard Siegwart controller
            vu = self.Krho * rho
            omega = self.Kalpha * alpha + self.Kbeta * beta
        else:
            # Extended with backward motion
            if -math.pi / 2 < alpha <= math.pi / 2:
                vu1 = self.Krho * rho
                omega1 = self.Kalpha * alpha + self.Kbeta * beta
                vu = self.constantSpeed
                omega = vu * omega1 / (vu1 + 1e-6)  # avoid div 0
            else:
                vu = -self.Krho * rho
                omega = self.Kalpha * alpha + self.Kbeta * beta

        twist.linear.x = vu
        twist.angular.z = omega
        return twist, rho

    def control_loop(self):
        if self.goal_received:
            cmd, rho = self.control_law()
            # Stop if close enough to goal
            if rho < 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SiegwartController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

