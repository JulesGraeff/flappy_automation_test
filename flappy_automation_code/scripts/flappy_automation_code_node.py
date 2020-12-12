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
VEL_INCREMENT = 0.24

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
        self.current_vel = Vector3(0.0,0.0,0.0)
        self.flappy_on_the_move = False

    def _vel_callback (self, msg):

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback

        # rospy.loginfo(rospy.get_caller_id() + f' msg from vel = {msg}')

        self.current_vel =  msg
        
        # x = 0
        # y = 0
        # self._pub_acc_cmd.publish(Vector3(x,y,0))

    def _laser_scan_callback (self, msg):

        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        i = 0

        for (one_laser_measurment, one_laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['dist_measured'] = one_laser_measurment
            self.lidar_dict[f'laser_{i}']['intensity'] = one_laser_intensity
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            i+=1

        assert self.lidar_dict['laser_8']['angle'] == msg.angle_max, f"Issue regarding angle increment ! Last laser angle [{self.lidar_dict[f'laser_8']['angle']}] != angle_max [{msg.angle_max}]"

        # for (laser_name, laser_characteristics) in zip(self.lidar_dict.keys(), self.lidar_dict.values()):

        if self.lidar_dict['laser_4']['dist_measured'] < LIMIT_DISTANCE:

            rospy.logwarn(rospy.get_caller_id() + f' An obstacle has been detected in front of Flappy !!')

            self.stop_flappy()
        
        elif not self.flappy_on_the_move:

            self.move_flappy_forward()

    def stop_flappy (self):

        self._pub_acc_cmd.publish(
            Vector3(
                -self.current_vel.x,
                -self.current_vel.y,
                -self.current_vel.z,
            )
        )

        self.flappy_on_the_move = False

    def move_flappy_forward (self):

        self._pub_acc_cmd.publish(
            Vector3(VEL_INCREMENT,0,0)
        )
        self.flappy_on_the_move = True

######################################################################################################################################################

if __name__ == '__main__':

    try:

        flappy_automation_node = FlappyAutomationNode()

        # Ros spin to prevent program from exiting
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################