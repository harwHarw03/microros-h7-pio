import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
import tf_transformations
import numpy as np
from tf2_ros import TransformBroadcaster
import threading

class ImuOdometryNode(Node):

    def __init__(self):
        super().__init__('imu_odometry_node')

        self.declare_parameter('gravity', 9.81)
        self.gravity_ = self.get_parameter('gravity').get_parameter_value().double_value
        self.footstep_distance_ = self.declare_parameter('footstep_distance', 0.5).value
        self.visualization_rate_ = self.declare_parameter('visualization_rate', 30.0).value

        self.position_ = np.array([0.0, 0.0, 0.0])
        self.velocity_ = np.array([0.0, 0.0, 0.0])
        self.orientation_ = np.array([0.0, 0.0, 0.0, 1.0]) # (x, y, z, w)
        self.last_time_ = None
        self.last_footstep_position_ = np.copy(self.position_)
        self.latest_stamp_ = self.get_clock().now().to_msg()
        self.data_lock_ = threading.Lock()

        self.imu_sub_ = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        self.odom_pub_ = self.create_publisher(Odometry, 'imu/odometry', 10)
        self.path_pub_ = self.create_publisher(Path, 'imu/path', 10)
        self.footstep_pub_ = self.create_publisher(MarkerArray, 'imu/footsteps', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.path_msg_ = Path()
        self.footstep_markers_ = MarkerArray()
        self.footstep_id_ = 0
        
        self.visualization_timer_ = self.create_timer(
            1.0 / self.visualization_rate_, 
            self.visualization_callback
        )

        self.get_logger().info(f'IMU Odometry Node started. Visualizations will be published at {self.visualization_rate_} Hz.')
        self.get_logger().warning('NOTE: Odometry from raw IMU integration drifts significantly over time.')

    def imu_callback(self, msg: Imu):

        with self.data_lock_:
            current_time = Time.from_msg(msg.header.stamp)
            self.latest_stamp_ = msg.header.stamp

            self.orientation_ = np.array([
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ])

            if self.last_time_ is not None:
                dt = (current_time - self.last_time_).nanoseconds / 1e9
                if dt <= 0: return

                linear_accel = np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])
                rotation_matrix = tf_transformations.quaternion_matrix(self.orientation_)[:3, :3]
                accel_world = rotation_matrix.dot(linear_accel)
                accel_world[2] -= self.gravity_

                self.velocity_ += accel_world * dt
                self.position_ += self.velocity_ * dt

            self.last_time_ = current_time

    def visualization_callback(self):

        with self.data_lock_:
            stamp = self.latest_stamp_
            position = np.copy(self.position_)
            orientation = np.copy(self.orientation_)
            velocity = np.copy(self.velocity_)

        self.publish_odometry(stamp, position, orientation, velocity)
        self.publish_tf(stamp, position, orientation)
        self.publish_path(stamp, position)
        self.publish_footsteps(stamp, position)

    def publish_odometry(self, stamp, position, orientation, velocity):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "imu_link"
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = position
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = orientation
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z = velocity
        self.odom_pub_.publish(odom_msg)

    def publish_tf(self, stamp, position, orientation):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "imu_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = orientation
        self.tf_broadcaster_.sendTransform(t)

    def publish_path(self, stamp, position):
        self.path_msg_.header.stamp = stamp
        self.path_msg_.header.frame_id = "odom"
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "odom"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        self.path_msg_.poses.append(pose)
        if len(self.path_msg_.poses) > 500: self.path_msg_.poses.pop(0)
        self.path_pub_.publish(self.path_msg_)

    def publish_footsteps(self, stamp, position):
        distance_moved = np.linalg.norm(position - self.last_footstep_position_)
        if distance_moved > self.footstep_distance_:
            self.last_footstep_position_ = np.copy(position)
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "footsteps"
            marker.id = self.footstep_id_
            self.footstep_id_ += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.pose.position.z -= 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.1, 0.02
            marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.8, 0.2, 0.8, 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=300).to_msg()
            self.footstep_markers_.markers.append(marker)
            self.footstep_pub_.publish(self.footstep_markers_)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
