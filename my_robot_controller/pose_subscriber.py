#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        
        #create a subscriber object
        #the 10 is the queue size, which is the maximum number of messages that can be stored in the queue before messages are dropped off during a traffic congestion
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
    
    def pose_callback(self,msg:Pose):
        self.get_logger().info("[subscriber]: (" + str(msg.x) + "," + str(msg.y) + ")")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()