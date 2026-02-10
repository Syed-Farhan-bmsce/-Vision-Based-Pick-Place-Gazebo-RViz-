#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time

class AutoBot(Node):
    def __init__(self):
        super().__init__('auto_bot')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.timer = self.create_timer(0.05, self.loop)
        self.start_time = time.time()

    def move_robot(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = positions
        self.joint_pub.publish(msg)

    def send_gripper(self, cmd):
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

    def loop(self):
        elapsed = time.time() - self.start_time
        
        # Phase 1: Approach (0-5s)
        if elapsed < 5.0: 
            self.move_robot([0.0, 0.5, 0.0, 1.0, 0.0, 1.5, 0.0])
            
        # Phase 2: Pick (5-6s)
        elif elapsed < 6.0: 
            self.send_gripper("GRASP")
            
        # Phase 3: Lift & Rotate (6-10s)
        elif elapsed < 10.0: 
            self.move_robot([1.57, -0.5, 0.0, 0.5, 0.0, 0.0, 0.0])
            
        # Phase 4: Place (10-11s)
        elif elapsed < 11.0: 
            self.send_gripper("RELEASE")
            
        # Phase 5: Home
        else: 
            self.move_robot([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def main():
    rclpy.init()
    node = AutoBot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
