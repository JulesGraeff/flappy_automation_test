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

CLOSED_DISTANCE = 2.6
LETHAL_DISTANCE = 1.0
VEL_INCREMENT = 0.2

class FlappyAutomationNode:

    def __init__ (self):

        self.node_name = 'flappy_automation_code'

        # Here we initialize our node running the automation code
        rospy.init_node(self.node_name, anonymous = True)
        rospy.loginfo(rospy.get_caller_id() + '_ "' + self.node_name + '" node has been created.')

        # Publisher for sending acceleration commands to flappy bird
        self._pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

        # Subscribe to topics for self.current_vel.x and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self._vel_callback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan, self._laser_scan_callback)

        self.lidar_dict = dict()
        self.current_vel = Vector3(0.0,0.0,0.0)
        self.flappy_on_the_move = dict(forward = False, up = False, down = False)
        self.game_is_started = False
        self.front_lasers_measurement_list = [CLOSED_DISTANCE]

        while not self.game_is_started:
            pass

        while not rospy.is_shutdown():

            try:

                if min(self.front_lasers_measurement_list) < CLOSED_DISTANCE:

                    self.find_free_path()

                if min(self.front_lasers_measurement_list) < LETHAL_DISTANCE:

                    rospy.logwarn(rospy.get_caller_id() + f' An obstacle has been detected in front of Flappy !!')
                    self.stop_flappy()
                
                elif not self.flappy_on_the_move['forward'] :
                    self.move_flappy_forward()

            except rospy.ROSInterruptException:
                break
        

    def _vel_callback (self, msg):

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on self.current_vel.x velCallback

        self.current_vel =  msg

        if True in self.flappy_on_the_move.values():
            self.stabilize_flappy_velocity()

    def _laser_scan_callback (self, msg):

        self.game_is_started = True

        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        i = 0

        self.front_lasers_measurement_list.clear()

        for (laser_measurment, laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['dist_measured'] = laser_measurment
            self.lidar_dict[f'laser_{i}']['number'] = i
            self.lidar_dict[f'laser_{i}']['intensity'] = laser_intensity
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            if i in [3, 4, 5]:
                self.front_lasers_measurement_list.append(laser_measurment)

            i+=1

        assert self.lidar_dict['laser_8']['angle'] == msg.angle_max, f"Issue regarding angle increment ! Last laser angle [{self.lidar_dict[f'laser_8']['angle']}] != angle_max [{msg.angle_max}]"

    def stop_flappy (self):

        self._pub_acc_cmd.publish(
            Vector3(
                -self.current_vel.x,
                -self.current_vel.y,
                -self.current_vel.z,
            )
        )

        self.flappy_on_the_move['forward'] = False
        self.flappy_on_the_move['up'] = False
        self.flappy_on_the_move['down'] = False

    def move_flappy_forward (self):

        self._pub_acc_cmd.publish(
            Vector3(
                VEL_INCREMENT,
                0.0,
                0.0
            )
        )
        self.flappy_on_the_move['forward'] = True

    def move_flappy_up (self):

        self._pub_acc_cmd.publish(
            Vector3(
                0.0,
                VEL_INCREMENT,
                0.0
            )
        )
        self.flappy_on_the_move['up'] = True

    def move_flappy_down (self):

        self._pub_acc_cmd.publish(
            Vector3(
                0.0,
                - VEL_INCREMENT,
                0.0
            )
        )
        self.flappy_on_the_move['down'] = True

    def stabilize_flappy_velocity (self):
        
        if math.fabs(self.current_vel.x) > VEL_INCREMENT:

            # print(f'TOO FAST Vx= {self.current_vel.x}')

            self._pub_acc_cmd.publish(
                Vector3(
                    -(math.fabs(self.current_vel.x) - VEL_INCREMENT),
                    0.0,
                    0.0
                )
            )

        if math.fabs(self.current_vel.y) > VEL_INCREMENT:

            if self.current_vel.y > 0:
                compensation = -(math.fabs(self.current_vel.y) - VEL_INCREMENT)
            else:
                compensation = (math.fabs(self.current_vel.y) - VEL_INCREMENT)

            self._pub_acc_cmd.publish(
                Vector3(
                    0.0,
                    compensation,
                    0.0
                )
            )

    def find_free_path (self):

        # list_laser 
        
        # if True in [laser['intensity'] == 0.0 in self.lidar_dict.values()]:

        no_obstacle_detected_list = []

        for laser in self.lidar_dict.values():

            if laser['intensity'] == 0.0:
                print(f"Laser number {laser['number']} is not encountring any cloosed obstacle")

                no_obstacle_detected_list.append(laser['number'])

        free_laser_number_mean = 4

        if len(no_obstacle_detected_list) != 0:
            free_laser_number_mean = sum(no_obstacle_detected_list) / len(no_obstacle_detected_list)

        print(f'free_laser_number_mean = {free_laser_number_mean} ')

        if free_laser_number_mean > 4:

            print('UP')

            if not self.flappy_on_the_move['up']:

                self.move_flappy_up()

        elif free_laser_number_mean < 4:
            print('DOWN')

            if not self.flappy_on_the_move['down']:

                self.move_flappy_down()

        else:
            print('STAND BY')

            

        # theta = 

        # math.tan(((math.pi/2) - theta))

######################################################################################################################################################

if __name__ == '__main__':

    try:

        flappy_automation_node = FlappyAutomationNode()

        # Ros spin to prevent program from exiting
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################