#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class SmartObject(Node):
    def __init__(self):
        super().__init__('smart_object')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscription = self.create_subscription(String, '/gripper_command', self.gripper_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        # Object State
        self.attached = False
        self.x, self.y, self.z = 0.5, 0.0, 0.0  # Initial Position on Table

    def gripper_callback(self, msg):
        command = msg.data
        if command == "GRASP":
            self.attached = True
            self.get_logger().info('Object ATTACHED to Gripper')
        elif command == "RELEASE":
            self.attached = False
            self.x, self.y, self.z = 0.0, 0.5, 0.0  # New Drop Position
            self.get_logger().info('Object RELEASED')

    def publish_marker(self):
        marker = Marker()
        # If attached, frame is 'link_eef' (end effector). If not, 'link_base'.
        marker.header.frame_id = "link_eef" if self.attached else "link_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "smart_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Scale (Size of the cube)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Color (Red)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Position
        if self.attached:
            marker.pose.position.x = 0.05  # Offset from gripper
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
        else:
            marker.pose.position.x = self.x
            marker.pose.position.y = self.y
            marker.pose.position.z = self.z

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SmartObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
