#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()

        #1.Straight line
        vel.linear.x = 0.5
        vel.angular.z = 0.01
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(82):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
        #2.Rotation 
        vel.linear.x = 0
        vel.angular.z = 0.2
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(80):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
	#3.Straight line
	vel.linear.x = 0.5
        vel.angular.z = 0.01
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(82):
            self.vel_pub.publish(vel)
            self.rate.sleep()
	
	#4.Rotation
        vel.linear.x = 0
        vel.angular.z = 0.2
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(80):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        

	#5.Straight line
	vel.linear.x = 0.5
        vel.angular.z = 0.01
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(82):
            self.vel_pub.publish(vel)
            self.rate.sleep()
		
	#6.Rotation
        vel.linear.x = 0
        vel.angular.z = 0.2
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(70):
            self.vel_pub.publish(vel)
            self.rate.sleep()
	
	#7.Straight line
	vel.linear.x = 0.5
        vel.angular.z = 0.01
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(71):
            self.vel_pub.publish(vel)
            self.rate.sleep()
		


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
