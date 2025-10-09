import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance
from coppeliasim_zmqremoteapi_client import *
from scipy.spatial.transform import Rotation as R


class SubscriberVelocity(Node):
    def __init__(self):
        super().__init__('subscriber_velocity')
        self.subscription = self.create_subscription(Twist, 'myRobot/cmd_vel', self.callback, 10)
        self.wheel_base = 0.331     # meters
         # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim successfully.')

            # Get robot and motor handles
            self.robotHandle = self.sim.getObject('/myRobot')
            self.leftMotor = self.sim.getObject('/leftMotor')
            self.rightMotor = self.sim.getObject('/rightMotor')

            self.sim.startSimulation()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

    def callback(self, msg):
        linVel = msg.linear.x
        rotVel = msg.angular.z
        # Inverse kinematics
        rightVel = (linVel + self.wheel_base /2 * rotVel)
        leftVel  = (linVel - self.wheel_base /2 * rotVel)
        self.sim.setJointTargetVelocity(self.rightMotor,rightVel)
        self.sim.setJointTargetVelocity(self.leftMotor,leftVel)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
