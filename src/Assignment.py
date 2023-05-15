#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np



class Assignment():
    def __init__(self):
      node_name = "Assignment"
      self.startup = True
      self.turn = False
      
      self.ctrl_c = False
      self.turn = False

      rospy.init_node(node_name, anonymous=True)
      self.rate = rospy.Rate(10)

      self.vel = 0.1
      self.dist = 0.3


      self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      self.publisher_rate = rospy.Rate(10) # Hz
      self.vel_cmd = Twist()

      self.posx = 0.0
      self.posy = 0.0
      self.yaw = 0.0
      self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)

      self.min_distance = 0.0
      self.closest_object_position = 0.0 # degrees
      self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 
    
      rospy.on_shutdown(self.shutdownhook)

      rospy.loginfo(f"the {node_name} node has been initialised...")


    def shutdownhook(self):
        self.set_move_cmd(linear = 0,angular=0)
        self.publish
        self.ctrl_c = True

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()


    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)




    def laserscan_cb(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.min_distance = front_arc.min()
        arc_angles = np.arange(-20, 21)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]











    def main_loop(self):
        


        while not self.ctrl_c:
    
            #work out where the closest object currently is.
            self.closest_object = self.min_distance
            self.closest_object_location = self.closest_object_position

            if self.turn:
            
                if self.closest_object > self.dist:
                    self.set_move_cmd(linear = 0,angular=0)
                    self.turn = False
                else: 
                    self.set_move_cmd(linear = 0,angular=0.1)
            


            if self.closest_object < self.dist:
                self.set_move_cmd(linear = 0,angular=0)
                self.turn = True
            
            
            self.set_move_cmd(linear = 0.1,angular=0)
            self.publish

if __name__ == "__main__":
    node = Assignment()
    node.__init__()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass



