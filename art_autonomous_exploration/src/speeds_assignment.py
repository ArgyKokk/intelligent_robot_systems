#!/usr/bin/env python

import rospy
import math
import time

import numpy as np

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.1), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)
      self.print_velocities = True
      # Print the speeds for debuggind purposes
      # if self.print_velocities == True:
      print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
	  # Create an array of all lidar measurements
      scan = np.array(self.laser_aggregation.laser_scan)
      linear  = 0
      angular = 0
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance      
      
      #find min-max laser angle using the ROS LaserScan attributes
      angle_min = self.laser_aggregation.angle_min
      angle_max = self.laser_aggregation.angle_max
      
      #create an array with the angles of all laser beams
    
      angles = np.linspace(angle_min, angle_max, len(scan))
 
      # cosine for frontal rays
      linear = -sum(np.cos(angles)/scan**2)/len(scan)
      # sine for side rays
      angular = -sum(np.sin(angles)/scan**2)/len(scan)
      
      
      
      
      
      #print "min angle" + str(angle_min/3.14*180)
      #print "max angle" +str(angle_max/3.14*180)      
      #print "len " +str(len(scan))
      #mindis = scan[0]
      #maxdis = scan[0]
      #minbeam = 0
      #maxbeam = 0
      #for i in range(0, len(scan)):
		 # if scan[i] < mindis:
		#	  mindis = scan[i]
	    #		  minbeam = i
		 # if scan[i] > maxdis:
			#  maxdis = scan[i]
			 # maxbeam = i
      
              
      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      # weigthts for target speeds
      w_linear = 0.15
      w_angular = 0.2
      
      #print "FLAG " +str(self.navigation.TimeOut)
      if self.move_with_target == True and self.navigation.TimeOut == False :
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
        
        # in motor schema we combine inputs by applying different weights
        # our first priority is to avoid obstacles and after that to reach the target
        # the speeds for obstacle avoidance will have a bigger weight
        self.linear_velocity = l_goal + w_linear*l_laser
        self.angular_velocity = a_goal + w_angular * a_laser
        
          
        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        
        self.linear_velocity =0.2 + 0.3*l_laser
        self.angular_velocity = 0.3*a_laser   
        if self.navigation.counter_to_next_sub == 100  :
			self.navigation.TimeOut = False
			self.navigation.Reset = True
			print "RESET"
        pass
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
