#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.create_timer(2.0 ,self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Hello from ROS " + str(self.counter))
        self.counter+=1
        
        
def main(args=None):
    #we initialize the ros client library with the name of the node
    #start ros communication
    rclpy.init(args=args)
    
    #we create an instance of the node
    node = MyNode()
    
    #we keep the node running till we kill it with ctrl+c
    rclpy.spin(node)
    
    
    
    
    #we create a node with the name my_first_node 
    #terminate a ros node
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()