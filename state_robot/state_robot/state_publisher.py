import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter('wheel_separation', 0.5)
        self.declare_parameter('wheel_radius', 0.1)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0

        self.joint_state = JointState()
        self.joint_state.name = ['drivewhl_r_joint', 'drivewhl_l_joint']
        self.joint_state.position = [self.right_wheel_pos, self.left_wheel_pos]


    def listener_callback(self, msg):
        print('I heard: [%s]' % msg)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        self.left_wheel_pos += left_wheel_vel
        self.right_wheel_pos += right_wheel_vel

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]

        self.publisher_.publish(self.joint_state)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['drivewhl_r_joint', 'drivewhl_l_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]

        transform.transform.translation.x = cos(self.left_wheel_pos) * 0.2  # Just an example, update accordingly
        transform.transform.translation.y = sin(self.right_wheel_pos) * 0.2  # Just an example, update accordingly
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
