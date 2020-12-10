#!/usr/bin/env python

#####################################################################################################################################################

# @project        IROS Partiel
# @file           path/to/file
# @author         Jules Graeff

######################################################################################################################################################

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

######################################################################################################################################################

class FlappyAutomationNode:

    def __init__ (self):

        self.node_name = 'flappy_automation_code'

        # Here we initialize our node running the automation code
        rospy.init_node(self.node_name, anonymous = True)
        rospy.loginfo(rospy.get_caller_id() + '_ "' + self.node_name + '" node has been created.')

        # Publisher for sending acceleration commands to flappy bird
        self._pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self._vel_callback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan, self._laser_scan_callback)

    def _vel_callback (self, msg):

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        x = 0
        y = 0
        self._pub_acc_cmd.publish(Vector3(x,y,0))

    def _laser_scan_callback (self, msg):
        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        rospy.loginfo(f"Laser range: {msg.ranges[0]}, angle: {msg.angle_min}")

######################################################################################################################################################

if __name__ == '__main__':

    try:
        flappy_automation_node = FlappyAutomationNode()

        # Ros spin to prevent program from exiting
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################