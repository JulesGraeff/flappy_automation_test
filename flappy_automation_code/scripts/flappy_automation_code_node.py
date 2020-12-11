#!/usr/bin/env python

#####################################################################################################################################################

# @project        IROS Partiel
# @file           path/to/file
# @author         Jules Graeff

######################################################################################################################################################

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

######################################################################################################################################################

LIMIT_DISTANCE = 1.8

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

        self.lidar_dict = dict()

    def _vel_callback (self, msg):

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        x = 0
        y = 0
        self._pub_acc_cmd.publish(Vector3(x,y,0))

    def _laser_scan_callback (self, msg):

        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        i = 0

        for (one_laser_measurment, one_laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['measure'] = one_laser_measurment
            self.lidar_dict[f'laser_{i}']['intensity'] = one_laser_intensity
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            i+=1

        for (laser_name, laser_characteristics) in zip(self.lidar_dict.keys(), self.lidar_dict.values()):

            if laser_characteristics['intensity'] == 1.0:

                print(f'** {laser_name} is detecting an obstacle !')


            # print(one_laser_measurment)

            # if one_laser_measurment < LIMIT_DISTANCE:
            #     rospy.logwarn(rospy.get_caller_id() + f'- You are to close !!!')

        rospy.loginfo(self.lidar_dict)

        # rospy.loginfo(f"Laser range: {msg.ranges[0]}, angle: {msg.angle_min}")




######################################################################################################################################################

if __name__ == '__main__':

    try:
        flappy_automation_node = FlappyAutomationNode()

        # Ros spin to prevent program from exiting
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################