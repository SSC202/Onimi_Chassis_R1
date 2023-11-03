import rclpy
from rclpy.node import Node
import transforms3d as tfs
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class Chassis_Speed_Node(Node):
    def __init__(self,name):
        super().__init__(name)

        self.vx = 0.0
        self.vy = 0.0
        self.wc = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
    
        self.current_time = self.get_clock().now().to_msg().sec
        self.last_time = self.get_clock().now().to_msg().sec

        self.chassis_publisher = self.create_publisher(Odometry,"odom",10)
        self.chassis_subscriber = self.create_subscription(Float32MultiArray,"chassis",self.chassis_subscriber_callback,10)
        self.timer = self.create_timer(0.1,self.timer_callback)
    
    def chassis_subscriber_callback(self,msg):
        vx = msg.data[0]
        vy = msg.data[1]
        wc = msg.data[2]
        # self.get_logger().info(f"Chassis speed:{vx},{vy},{wc}")
        self.vx = vx * math.cos(self.theta) - vy * math.sin(self.theta)
        self.vy = vx * math.sin(self.theta) + vy * math.cos(self.theta)
        self.wc = wc

    def timer_callback(self):
        self.current_time = self.get_clock().now().to_msg().sec
        dt = self.current_time - self.last_time

        delta_x = self.vx * dt
        delta_y = self.vy * dt
        delta_theta = self.wc * dt

        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.theta = self.theta + delta_theta

        x,y,z,w = tfs.euler.euler2quat(0,0,self.theta,"sxyz")

        if (self.theta > math.pi):
            self.theta = self.theta - 2 * math.pi

        if (self.theta < - math.pi):
            self.theta = self.theta + 2 * math.pi
        
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = x
        odom_msg.pose.pose.orientation.y = y
        odom_msg.pose.pose.orientation.z = z
        odom_msg.pose.pose.orientation.w = w

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.wc

        self.chassis_publisher.publish(odom_msg)
        self.last_time = self.current_time

def main(args = None):
    rclpy.init(args=args)
    node = Chassis_Speed_Node("chassis_node")
    rclpy.spin(node)
    rclpy.shutdown()

