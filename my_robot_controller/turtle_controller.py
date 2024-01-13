#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x = 0.0
        self.get_logger().info("Turtle Controller has started")
        self.pose_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose",self.pose_callback,10)
     
    def pose_callback(self, msg:Pose):
        command = Twist()
        
        #maintain boundaries
        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y<2.0:
            command.linear.x = 1.0
            command.angular.z = 0.9
        else:
            command.linear.x = 5.0
            command.angular.z = 0.0
        
        #set pen color
        if msg.x > 5.5 and self.previous_x <= 5.5:
            self.set_pen_service(255,77,77,2,0)
            self.get_logger().info("COLOR: RED")
            self.previous_x = msg.x
        elif msg.x < 5.5 and self.previous_x >= 5.5:
            self.set_pen_service(0,199,199,2,0)
            self.get_logger().info("COLOR: TEAL")
            self.previous_x = msg.x
        
        #publish the command
        self.pose_pub.publish(command)
        
    def set_pen_service(self, r,g,b,width,off):
        client = self.create_client(SetPen,"/turtle1/set_pen")
        #wait for service to be ready
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Service not yet available, waiting...")
        #create request
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off 
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self,future):
        try:
            response = future.result()
            #we do not need any response because it's a void service
            self.get_logger().info("Pen set") 
        except Exception as e:
            self.get_logger().error("Service call failed %r",(e,))
def main(args = None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()