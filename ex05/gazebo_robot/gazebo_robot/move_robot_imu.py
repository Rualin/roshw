#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Прямолинейное движение вперед
        self.move_forward()
        time.sleep(3)
        
        # Поворот
        self.turn_left()
        time.sleep(2)
        
        # Остановка
        self.stop()
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # 0.5 м/с
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Moving forward')
    
    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.5  # поворот налево
        self.publisher.publish(msg)
        self.get_logger().info('Turning left')
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin_once(node, timeout_sec=5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()