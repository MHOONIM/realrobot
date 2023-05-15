#!/usr/bin/env python3

import rospy
import roslaunch


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from tf.transformations import euler_from_quaternion
from time import sleep
import roslaunch
import rospkg
from pathlib import Path

from math import sqrt,pow,pi

class Assignment():

    #callback function  for the odometry
    def callback_function(self,topic_data:Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        pos_x = position.x
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        self.x = pos_x
        self.y = pos_y
        self.theta_z = abs(yaw)

        if self.startup:
            # don't initialise again:
            self.startup = False
        
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z


    #callback function for the scanning. SOMETIMES min distance refuses to work
    #think i fixed it, but don't quote me on that.
    def scan_callback(self, scan_data):
        
        
        #this sets the range for the obstacle avoidance on the left and right
        #these need to be larger so it can keep moving whilst avoiding objects
        left_arc = scan_data.ranges[0:41]
        right_arc = scan_data.ranges[-40:]
        

        #this sets the arc for stopping and turning
        #this is minimal, but enough so that a head on collision is avoided
        #basically; collision avoidance.
        frontleft = scan_data.ranges[0:21]
        frontright = scan_data.ranges[-20:]
        
        #the values are combined to make front_arc
        front_arc = np.array(frontleft+ frontright)
       
        #min_distance is the min distance of just the FRONT ARCs
        #if this is too small then the system has to stop and turn.
        self.min_distance=front_arc.min()
        

        #then we set up the left and right arcs for obstacle avoidance.
        left = np.array(left_arc)
        right = np.array(right_arc)
        



        self.min_left = left.min()
        self.min_right = right.min()
    
        if self.min_distance <= 0.01:
            self.min_distance = 3
        if self.min_left <= 0.01:
            self.min_left = 3
        if self.min_right <= 0.01:
            self.min_right = 3




    #initalise it all!
    def __init__(self):
        self.min_distance = 0 
        node_name = "Assignment"
        self.startup = True

        self.turn = False
        self.leftside = False
        self.rightside = False   

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        self.subscan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz
        self.time = rospy.get_time()
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


        self.min_left = 0
        self.min_right = 0

        rospy.loginfo(f"the {node_name} node has been initialised...")


        pkg_path = rospkg.RosPack().get_path('realrobot')
        map_path = Path(pkg_path).joinpath("explore_map")
        map_path.mkdir(exist_ok=True)
        self.map_file = map_path.joinpath('gr5_map')

        self.ros_l = roslaunch.scriptapi.ROSLaunch()
        self.ros_l.start()        
        rospy.loginfo(f"the Map is initialised...")


    #ensuring a control c shuts the fella down
    def shutdownhook(self):
      
        self.pub.publish(Twist())
        self.ctrl_c = True




    def main_loop(self):
        

        #these three variables relate to collision avoidance
        #the distance at which it needs to turn
        #the distance at which it can stop
        #and the turning speed
        DangerDistance = 0.55
        SafeDistance = 0.9
        TurnSpeed = 0.8


        #these three variables relate to obstacle avoidance.
        #the distance at which it needs to turn
        #the distance at which it can stop
        #and the turning speed
        ObstacleDistance = 0.9
        ObstacleGone = 1.3
        ObstacleTurnSpeed = 0.3

        #so whilst control c is not pressed
        while not self.ctrl_c:



            #if it has to turn, it turns
            #once the safe distance has been reach. it stops and say so
            if self.turn: 
                if self.min_distance > SafeDistance:
                    self.vel = Twist()
                    self.turn = False
                    rospy.loginfo("I'm done turning!")

                else:
                    self.vel.angular.z = TurnSpeed
                    self.vel.linear.x = -0.01
                


            #if it doesn't need to turn
            else:

                if self.min_distance < DangerDistance:
                    self.vel = Twist()
                    self.turn = True
                    
                    #if it does, it checks which side is closer

                    #if the left side is closer than the right
                    #it will turn away from the left

                    #if the right is closer than the left
                    #it will turn away from the right
                    if self.min_left < self.min_right:
                            TurnSpeed = -TurnSpeed 
                    elif self.min_right < self.min_left:
                            TurnSpeed = TurnSpeed 
                                        #and it lets the user know
                    rospy.loginfo("Obsticle Too Close! Stop and turn!")
                else:
                       self.vel.linear.x = 0.26

            

         
                    #if its too close to the left it sets a flag
                if self.min_left < ObstacleDistance:
                    self.vel.angular.z = -ObstacleTurnSpeed
                    rospy.loginfo("Obsticle On the Left!")
                    self.leftside = True
                elif self.min_left < DangerDistance:
                    self.vel = Twist()
                    self.turn = True

                if self.min_right < ObstacleDistance:
                    self.vel.angular.z = ObstacleTurnSpeed
                    rospy.loginfo("Obsticle On the Right!")
                    self.rightside = True 
                elif self.min_right < DangerDistance:
                    self.vel = Twist()
                    self.turn = True


                #which is resolved here
                if self.leftside:
                    if self.min_left > ObstacleGone:
                        self.leftside = False
                        self.vel.angular.z = 0
                        rospy.loginfo("Left Cleared!")
          
                #which is resolved here
                if self.rightside:
                    if self.min_right > ObstacleGone:
                        self.rightside = False
                        self.vel.angular.z = 0
                        rospy.loginfo("Right Cleared!")
                        
                #importantly, the left and right swerving can clash
                #causing it to 'jitter'...
                #this probably needs to be fixed, but im not sure how...

                #emergent behaviour results in a pretty set path.


            # publish whatever velocity command has been set
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()
            
            # print(f'rostime = {rospy.get_time()}')
            # Update the map after navigation in every 5 seconds
            if (rospy.get_time() - self.time) > 5:
                self.ros_l.launch(roslaunch.core.Node(
                        package="map_server",
                        node_type="map_saver",
                        args=f"-f {self.map_file}")) 
                self.time = rospy.get_time()    








if __name__ == "__main__":
    node = Assignment()
    node.__init__()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
