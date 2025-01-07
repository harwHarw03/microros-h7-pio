import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class ImuPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher_node')
        
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        self.current_time = self.get_clock().now()
        self.angular_velocity_z = 0.5  # rad/s
        self.linear_acceleration_x = 0.2 # m/s^2
        self.current_angle = 0.0

        self.get_logger().info('imu_publisher has been started.')

    def publish_imu_data(self):

        msg = Imu()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        dt = 0.1
        self.current_angle += self.angular_velocity_z * dt
        
        if self.current_angle > math.pi:
            self.current_angle -= 2 * math.pi
        elif self.current_angle < -math.pi:
            self.current_angle += 2 * math.pi
        
        q = self.euler_to_quaternion(0, 0, self.current_angle)
        msg.orientation.x = q.x
        msg.orientation.y = q.y
        msg.orientation.z = q.z
        msg.orientation.w = q.w
        
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = self.angular_velocity_z
        
        msg.linear_acceleration.x = self.linear_acceleration_x
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        
        for i in range(9):
            msg.orientation_covariance[i] = 0.0
            msg.angular_velocity_covariance[i] = 0.0
            msg.linear_acceleration_covariance[i] = 0.0

        self.publisher_.publish(msg)
        # self.get_logger().info(f'data  yaw: {self.current_angle:.2f}')

    def euler_to_quaternion(self, roll, pitch, yaw):

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    imu_publisher_node = ImuPublisherNode()
    rclpy.spin(imu_publisher_node)
    imu_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
