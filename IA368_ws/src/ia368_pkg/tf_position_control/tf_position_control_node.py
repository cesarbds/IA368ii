#!/usr/bin/env python3
import math
import rclpy
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SiegwartController(Node):
    def __init__(self):
        super().__init__("siegwart_position_controller")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.old_t = TransformStamped()
        self.last_timestamp = 0

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
        self.goal_received = False

        # Subscribers
        self.create_subscription(Odometry, "myRobot/odom", self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "myRobot/cmd_vel", 10)

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # TF2 listener timer
        self.create_timer(0.05, self.listen_tf_timer)

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

    #def goal_callback(self, msg: Pose2D):
        
    def listen_tf_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        to_frame_rel = 'object_45'
        from_frame_rel = 'camera_color_optical_frame'
        try:
            self.t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        self.goal_received = True

    def control_law(self):
        self.last_timestamp = self.t.header.stamp.sec + self.t.header.stamp.nanosec*1e-9
        old_timestamp = self.old_t.header.stamp.sec + self.old_t.header.stamp.nanosec*1e-9
        dx, dy = 0, 0
        if ((self.last_timestamp - old_timestamp) == 0.0):
            rho, alpha, beta = 0.0, 0.0, 0.0
        else:
            self.old_t = self.t
            dx = self.t.transform.translation.z# - self.robot_pose.x
            dy = - self.t.transform.translation.x# - self.robot_pose.y
            rho = math.hypot(dx, dy)  # distance to goal
        
        twist = Twist()

        twist.linear.x = math.sqrt(dx ** 2 + dy ** 2)#vu
        twist.angular.z = 0.5 * math.atan2(dy, dx)#omega
        return twist, rho

    def control_loop(self):
        if ((self.get_clock().now().nanoseconds*1e-9 - self.last_timestamp)>1.0):
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

