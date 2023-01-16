import math 
def neighbors(current):
    #define the list of 4 neighbors
    neighbors = [[-0.5, 0], [0, 0.5], [0.5, 0], [0, -0.5]] # up, down, right, left
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):	
    #heuristic diatance
    distancex = abs(goal[0] - candidate[0])
    distancey = abs(goal[1] - candidate[1])
    distance = distancex + distancey
    
    return distance

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 

    #creates an open list with the cost first and the point second
    open_list = []
    open_list = [(0,start)]
    
    #create an empty closed list that is initialized as empty
    closed_list = []

    #create a dictionary for the past costs
    past_cost = {}
    past_cost[start] = 0

    #create a dictionary of parents
    parent = {}
    parent[start] = None 
    #initialize a path to return
    path = []
    
    #while the open_list is not empty
    while open_list:
        #remove the current cheapest node from the open list and move it to closed 
     
        current = open_list.pop(0)[1]
        closed_list.append(current)
        #print(closed_list)

	#if the current is within the goal the return, we dont want to do anything
 	if current == goal:
            while current != start:
                path.append(current)
                current = parent[current]
            
            path.reverse()
            break
 
	#if the goal is in a obstacle return an error
        if goal in obstacles:
            print("ERROR: GOAL IS WITHIN AN OBSTACLE")
            return
    
        for nbr in neighbors(current):
            if nbr in obstacles:
                continue
            tentative_past_cost = past_cost[current] + 0.5     #if the cell is available, +1  
  
            #if the neighbor has not been previously visited we need to make a addition to the cost
            if nbr not in past_cost or tentative_past_cost < past_cost[nbr]:
                  
                    #update past cost with the lower value
                    past_cost[nbr] = tentative_past_cost
                    #set the parent of the previous nbr
                    parent[nbr] = current
                    #calculate the estimated total cost based on past cost and the heuristic
                    est_total_cost = past_cost[nbr] + heuristic_distance(nbr, goal)
                    #append to open list and sort based on the total cost
                    open_list.append((est_total_cost, nbr))
        open_list.sort()
     
    #print(parent)
    #start from the goal to get path
    
    print(path)

    return path

#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.previous_waypoint = [0,0]   ## (x,y)
        self.previous_velocity = [0,0]   ## velx, vely
        self.vel_ref = 0.3
        self.vel = Twist()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
	start = (0, 0) 
	goal = (4.5,0)
   	obstacles = [(1, 0), (1, -0.5), (2.5, 0), (2.5, 1), (3.5, -0.5), (4, -0.5)]
        waypoints = get_path_from_A_star(start, goal, obstacles)
	waypoints.append(goal)
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint      
        # next_waypoint is to help determine the velocity to pass current_waypoint
        #determine boundary conditions for the position for x and y

	#x pose
        px_start = self.previous_waypoint[0]
        px_end = current_waypoint[0]
        #y pose
        py_start = self.previous_waypoint[1]
        py_end = current_waypoint[1]

        #determine boundary  conditions for velocity
        #need to compute the angle of the robot in order to change the x,y velocities
        vx_start = self.previous_velocity[0]    #velocity*cos()
        vy_start = self.previous_velocity[1]    #velocity*sin()

        #decompose the velocity
        dx = next_waypoint[0] - current_waypoint[0]
        dy = next_waypoint[1] - current_waypoint[1]
        angle = atan2(dy,dx)


        if(current_waypoint[0] == 0 and current_waypoint[1] == 0 and next_waypoint[0] == 0 and next_waypoint[1] == 0):  
	#here the robot should stop moving forward since vx and vy take part in determining linear velocity
            vx_end = 0  
            vy_end = 0
        else:       
            vx_end = self.vel_ref*cos(angle)  
            vy_end = self.vel_ref*sin(angle)


        #solve the polynomial coefficient
        #going to get coefficients for x and coefficients for y
        T = 2
        Kp = 5
        coeff_x = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T)
        coeff_y = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T)
        #print(coeff_x)
        #print(coeff_y)
        #now we want the robot to move, we have the coefficients now which can be used to find the velocity.
        #The velocity can now be sent to twist for a specified range of time values hint: use for loop
        
        for i in range(0, 10*T):
            #send the velocity
            #we can play with t and extend the time to see what happens
            t = i * 0.1 
            #compute Vx_end, Vy_end; get vx vy, then we go from velocity to angle
            vx_end = np.dot([3*t**2, 2*t, 1, 0], coeff_x) # ([row] * [col]); t = i * 0,1 (choose 0 or 1)
            vy_end = np.dot([3*t**2, 2*t, 1, 0], coeff_y)  # np.dot is done since its a matrix multiplication
            theta = atan2(vy_end, vx_end)    #re-calculate angle theta
            dtheta = theta - self.pose.theta # delta theta = starting theta - current theta 
            #print(theta)

            #here we change theta to be within the range of Gazebo (-pi, pi)
            if (dtheta < -pi): 
                self.vel.angular.z = Kp*(dtheta + 2*pi)
            elif (dtheta > pi):
                self.vel.angular.z = Kp*(dtheta - 2*pi)
                
            else:
                self.vel.angular.z = Kp*dtheta
            
            self.vel.linear.x = sqrt(vx_end**2 + vy_end**2)

            self.vel_pub.publish(self.vel)
            	                            
            self.rate.sleep()
            
        #Update the previous values
        self.previous_waypoint = current_waypoint
        self.previous_velocity = [vx_end, vy_end]
        pass
 
        # implement a PID controller similar to lab 3 
        # 
        #now we go to the next waypoint working with the velocity

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        # T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        A = np.array([[p_start], [p_end], [v_start], [v_end]])
        B = np.array([[0, 0, 0, 1],[T**3, T**2, T, 1],[0, 0, 1, 0],[3*T**2, 2*T, 1, 0]])
        
	##coeff = B^-1 * A
	binv = np.linalg.inv(B)
        return((np.dot(binv, A)))
        
        

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()


