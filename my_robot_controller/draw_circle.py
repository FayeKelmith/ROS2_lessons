#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        #we create a publisher object and pass it the datatype from geometry_msgs plus the topic name
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        
        #timer
        self.timer = self.create_timer(1.0, self.send_velocity_command)
        #log info that the node has started 
        self.get_logger().info("Draw Circle node has started")
        
    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 3.0
        msg.angular.z = 1.8 
        
        #to publish the message we call the publish method on the publisher object
        self.cmd_vel_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    
    node = DrawCircleNode()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()