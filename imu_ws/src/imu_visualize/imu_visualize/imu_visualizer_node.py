import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from rclpy.duration import Duration

class ImuVisualizerNode(Node):

    def __init__(self):
        super().__init__('imu_visualizer_node')
        
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        
        self.marker_publisher_ = self.create_publisher(Marker, 'imu/marker', 10)
        
        self.get_logger().info('imu_visualizer node has been started.')

    def imu_callback(self, msg: Imu):

        marker = Marker()
        
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "imu_visualization"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation = msg.orientation
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        
        marker.lifetime = Duration(seconds=0.5).to_msg()
        
        self.marker_publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    imu_visualizer_node = ImuVisualizerNode()
    rclpy.spin(imu_visualizer_node)
    imu_visualizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
