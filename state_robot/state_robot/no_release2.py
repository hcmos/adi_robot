import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi

from adi_tmcl.msg import TmcInfo

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

        # IMU data subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.imu_subscription  # prevent unused variable warning

        self.tmc_info_subscription = self.create_subscription(
            TmcInfo,
            '/tmc_info_0',
            self.tmc_info_0_callback,
            10)
        self.tmc_info_subscription  # prevent unused variable warning

        self.tmc_info_subscription = self.create_subscription(
            TmcInfo,
            '/tmc_info_1',
            self.tmc_info_1_callback,
            10)
        self.tmc_info_subscription  # prevent unused variable warning


        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter('wheel_separation', 0.517)
        self.declare_parameter('wheel_radius', 0.2)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0

        self.joint_state = JointState()
        self.joint_state.name = ['drivewhl_r_joint', 'drivewhl_l_joint']
        self.joint_state.position = [self.right_wheel_pos, self.left_wheel_pos]

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.imu_orientation = None

        #initialize tmc_info
        self.tmc_info_0 = TmcInfo()
        self.tmc_info_0.board_voltage = 0.0
        self.tmc_info_0.status_flag = 0
        self.tmc_info_0.status = ""
        self.tmc_info_0.motor_num = 0
        self.tmc_info_0.velocity = 0.0
        self.tmc_info_0.position = 0
        self.tmc_info_0.torque = 0

        self.tmc_info_1 = TmcInfo()
        self.tmc_info_1.board_voltage = 0.0
        self.tmc_info_1.status_flag = 0
        self.tmc_info_1.status = ""
        self.tmc_info_1.motor_num = 0
        self.tmc_info_1.velocity = 0.0
        self.tmc_info_1.position = 0
        self.tmc_info_1.torque = 0

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate the wheel velocities
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        # Update the wheel positions
        dt = 0.1  # assuming a control loop running at 10Hz
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]

        self.publisher_.publish(self.joint_state)

        # Update robot's position
        dx = linear_vel * cos(self.th) * dt
        dy = linear_vel * sin(self.th) * dt
        dth = angular_vel * dt

        self.x += dx
        self.y += dy
        self.th += dth

        # Transform from world to odom
        world_transform = TransformStamped()
        world_transform.header.stamp = self.get_clock().now().to_msg()
        world_transform.header.frame_id = 'world'
        world_transform.child_frame_id = 'odom'

        world_transform.transform.translation.x = self.x
        world_transform.transform.translation.y = self.y
        world_transform.transform.translation.z = 0.0
        world_transform.transform.rotation.x = 0.0
        world_transform.transform.rotation.y = 0.0
        world_transform.transform.rotation.z = sin(self.th / 2.0)
        world_transform.transform.rotation.w = cos(self.th / 2.0)

        self.broadcaster.sendTransform(world_transform)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # transform.transform.translation.x = self.x
        # transform.transform.translation.y = self.y
        # transform.transform.translation.z = 0.0
        # transform.transform.rotation.x = 0.0
        # transform.transform.rotation.y = 0.0
        # transform.transform.rotation.z = sin(self.th / 2.0)
        # transform.transform.rotation.w = cos(self.th / 2.0)

        
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(transform)

        # IMU Transform (fixed to base_link)
        if self.imu_orientation is not None:
            imu_transform = TransformStamped()
            imu_transform.header.stamp = self.get_clock().now().to_msg()
            # imu_transform.header.frame_id = 'base_link'#動く
            imu_transform.header.frame_id = 'base_link'
            imu_transform.child_frame_id = 'imu_link'

            imu_transform.transform.translation.x = 0.0
            imu_transform.transform.translation.y = 0.0
            imu_transform.transform.translation.z = 0.3  # Assuming IMU is placed 0.1m above base_link
            #imu_transform.transform.rotation = 0#self.imu_orientation

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            self.broadcaster.sendTransform(imu_transform)

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation
    
    def tmc_info_0_callback(self, msg):
        self.tmc_info_0 = msg.postion * pi / 180 #deg to rad
    
    def tmc_info_1_callback(self, msg):
        self.tmc_info_1 = msg.postion * pi / 180


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

