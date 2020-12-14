#!/usr/bin/env python

#####################################################################################################################################################

# @project        IROS Partiel
# @file           path/to/file
# @author         Jules Graeff

######################################################### LIBRARIES #################################################################################

import rospy
import time
import numpy as np
from math import fabs

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

######################################################## GLOBAL CONSTANTS ###########################################################################

CLOSED_DISTANCE = 2.2 
FRONT_LETHAL_DISTANCE = 1.1
LATERAL_LETHAL_DISTANCE = 0.5
ACC_INCREMENT = 0.25  * 2
LATERAL_TICK = 0.07

######################################################## NODE CLASS #################################################################################

class FlappyAutomationNode:

    def __init__ (self):

        # NODE INIT

        self.node_name = 'flappy_automation_code'
        rospy.init_node(self.node_name, anonymous = True)
        rospy.loginfo(rospy.get_caller_id() + '_ "' + self.node_name + '" node has been created.')

        # TOPICS DEFINITION

        # Publisher for sending acceleration commands to flappy bird
        self._pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

        # Subscribe to topics for self.current_vel.x and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self._vel_callback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan, self._laser_scan_callback)

        # CLASS VARIABLES

        self.game_is_started = False
        self._display_counter = 0

        # Lidar information
    
        self.lidar_dict = dict()
        self.front_lasers_measurement_list = [CLOSED_DISTANCE]
        self.bottom_laser_measurement = CLOSED_DISTANCE
        self.top_laser_measurement = CLOSED_DISTANCE

        # Mouvement information

        self.current_vel = Vector3(0.0,0.0,0.0)
        
        self.flappy_on_the_move = dict(forward = False, up = False, down = False)
        
        self.command_acc_x = 0.2
        self.command_acc_y = 0.45
        self.vel_x_reached = False
        self.vel_y_reached = False

        self.trying_to_find_path = False

        # MAIN

        while not self.game_is_started:
            pass

        # self.move_flappy_forward()
        self._pub_acc_cmd.publish(
            Vector3(
                self.command_acc_x,
                self.command_acc_y,
                0.0
            )
        )

        while not self.vel_x_reached:
            pass

        print(f"MAIN {self.flappy_on_the_move['forward']}")

        while not rospy.is_shutdown():

            try:

                # X VEL MANAGEMENT

                if min(self.front_lasers_measurement_list) <= FRONT_LETHAL_DISTANCE:
                    self.stop_x_flappy_vel()

                elif not self.flappy_on_the_move['forward']:
                    self.move_flappy_forward()
                    
                # Y VEL MANAGEMENT
                
                if self.top_laser_measurement <= LATERAL_LETHAL_DISTANCE or self.bottom_laser_measurement <= LATERAL_LETHAL_DISTANCE:
                    
                    self.stop_y_flappy_vel()
                    self.trying_to_find_path = False

                    if self.top_laser_measurement <= LATERAL_LETHAL_DISTANCE and not self.bottom_laser_measurement <= LATERAL_LETHAL_DISTANCE:
                        self.move_flappy_down(LATERAL_TICK*2)

                    elif self.bottom_laser_measurement <= LATERAL_LETHAL_DISTANCE and not self.top_laser_measurement <= LATERAL_LETHAL_DISTANCE:
                        self.move_flappy_up(LATERAL_TICK*2)

                elif min(self.front_lasers_measurement_list) < 3.5:
                    
                    self.find_free_path()
                    self.trying_to_find_path = True

                # ACC COMMAND

                if self._display_counter > 1000:

                    rospy.logdebug(f'ACC command => for x = {self.command_acc_x} && for y = {self.command_acc_y}')
                    self._display_counter = 0

                self._pub_acc_cmd.publish(
                    Vector3(
                        self.command_acc_x,
                        self.command_acc_y,
                        0.0
                    )
                )

                self._display_counter += 1

            except rospy.ROSInterruptException:
                break

    def _vel_callback (self, vel_vector):

        self.current_vel = vel_vector

        if self.game_is_started:
        
            # VEL X STABILISATION

            if self.current_vel.x >= self.command_acc_x:

                if not self.vel_x_reached: # STOP X ACC

                    rospy.loginfo(f'X Speed reached for {self.current_vel.x}m/s.')
                    self.command_acc_x = 0.0
                    self.vel_x_reached = True

            else:
                self.vel_x_reached = False

            # TICK Y STABILISATION

            # if self.trying_to_find_path:

            if fabs(self.current_vel.y) >= fabs(self.command_acc_y):

                if not self.vel_y_reached: # STOP Y ACC

                    rospy.loginfo(f'Y tick reached.')
                    self.stop_y_flappy_vel()
                    self.vel_y_reached = True

            else:
                self.vel_y_reached = False

            # ON THE MOVE DEFINITION

            if fabs(self.current_vel.x) < 0.0005:

                self.flappy_on_the_move['forward'] = False
                self.command_acc_x = 0.0

            if fabs(self.current_vel.y) < 0.0005:

                self.flappy_on_the_move['up'] = False
                self.flappy_on_the_move['down'] = False
                self.command_acc_y = 0.0

    def _laser_scan_callback (self, msg):

        if not self.game_is_started:

            self.game_is_started = True
            self.time_init = time.time()

        i = 0
        self.front_lasers_measurement_list.clear()

        for (laser_measurement, laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['dist_measured'] = laser_measurement
            self.lidar_dict[f'laser_{i}']['number'] = i
            self.lidar_dict[f'laser_{i}']['intensity'] = laser_intensity
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            if i  == 0:
                self.bottom_laser_measurement = laser_measurement
            
            if i == 8:
                self.top_laser_measurement = laser_measurement

            if i in [3, 4, 5]:
                self.front_lasers_measurement_list.append(laser_measurement)

            i += 1

        assert self.lidar_dict['laser_8']['angle'] == msg.angle_max, f"Issue regarding angle increment ! Last laser angle [{self.lidar_dict[f'laser_8']['angle']}] != angle_max [{msg.angle_max}]"

        if self._display_counter > 1000:

            if self.top_laser_measurement <= LATERAL_LETHAL_DISTANCE:
                rospy.logwarn('TOP LASER ARE INFORMING OF A COLLISION!')

            if self.bottom_laser_measurement <= LATERAL_LETHAL_DISTANCE:
                rospy.logwarn('DOWN LASER ARE INFORMING OF A COLLISION!')

            if min(self.front_lasers_measurement_list) <= FRONT_LETHAL_DISTANCE:
                rospy.logwarn('FRONT LASER ARE INFORMING OF A COLLISION!')

    def stop_x_flappy_vel (self):
        self.command_acc_x = -self.current_vel.x

    def stop_y_flappy_vel (self):
        self.command_acc_y = -self.current_vel.y

    def move_flappy_forward (self, acc = ACC_INCREMENT):

        self.command_acc_x = acc
        self.flappy_on_the_move['forward'] = True

    def move_flappy_up (self, acc = LATERAL_TICK):

        self.command_acc_y = acc
        self.flappy_on_the_move['up'] = True
        self.flappy_on_the_move['down'] = False

    def move_flappy_down (self, acc = LATERAL_TICK):

        self.command_acc_y = -acc
        self.flappy_on_the_move['down'] = True
        self.flappy_on_the_move['up'] = False

    def find_free_path (self):

        free_path_detected_list = []

        for laser in self.lidar_dict.values():

            if laser['intensity'] == 0.0 or laser['dist_measured'] > CLOSED_DISTANCE:

                rospy.logdebug(f"Laser number {laser['number']} is not encountring any cloosed obstacle")
                free_path_detected_list.append(laser['number'])

        free_laser_number_mean = 4.0

        if len(free_path_detected_list) != 0:
            free_laser_number_mean = sum(free_path_detected_list) / len(free_path_detected_list)

        rospy.logdebug(f'free_laser_number_mean = {free_laser_number_mean} ')

        if free_laser_number_mean > 4.0:
            self.move_flappy_up()

        elif free_laser_number_mean < 4.0:
            self.move_flappy_down()

        else:

            # bottom_section = sum([self.lidar_dict[f'laser_{i}']['dist_measured'] for i in [0, 1, 2, 3]])
            # top_section = sum([self.lidar_dict[f'laser_{i}']['dist_measured'] for i in [5, 6, 7, 8]])

            # if bottom_section < top_section:
            #     self.move_flappy_up(LATERAL_TICK/2)

            # elif bottom_section > top_section:
            #     self.move_flappy_down(LATERAL_TICK/2)

            # else:
            self.stop_y_flappy_vel()


######################################################################################################################################################

if __name__ == '__main__':

    try:

        flappy_automation_node = FlappyAutomationNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################