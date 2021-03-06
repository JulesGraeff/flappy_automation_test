#!/usr/bin/env python

#####################################################################################################################################################

# @project        Test for FlyAbility (fork from https://github.com/JohsBL/flappy_automation_test)
# @file           src/flappy_automation_test/flappy_automation_code/scripts/flappy_automation_code_node.py
# @author         Jules Graeff

######################################################### LIBRARIES #################################################################################

import rospy
import time
import numpy as np
from math import fabs
from enum import Enum

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

######################################################## GLOBAL CONSTANTS ###########################################################################

SPEED_FACTOR = 2
CLOSED_DISTANCE = 2.15
FRONT_LETHAL_DISTANCE = 1.1
ACC_INCREMENT = 0.25  * SPEED_FACTOR
LATERAL_TICK = 0.07 * SPEED_FACTOR

class State(Enum):
    INIT = 0
    DOOR_SEARCH = 1
    DOOR_FOUND = 2
    GO_THROUGH_DOOR = 3
    PASSING_IN_DOOR = 4
    DOOR_PASSED = 5

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
        self.lidar_dict = dict()    # Lidar information

        # Mouvement information

        self.current_vel = Vector3(0.0,0.0,0.0)
        
        self.flappy_on_the_move = dict(forward = False, up = True, down = False)
        
        self.command_acc_x = ACC_INCREMENT
        self.command_acc_y = 0.5
        self.tick_y_reached = False

        state = State.INIT
        old_state = None

        self.obstacles_crossed_number = 0

        # MAIN

        while not self.game_is_started:
            pass

        while not rospy.is_shutdown():

            try:

                # STATE DEFINITION

                if state == State.DOOR_SEARCH and min(self._select_lasers('front', 'distance')) > CLOSED_DISTANCE:
                    state = State.DOOR_FOUND

                elif (min(self._select_lasers('front', 'distance')) < CLOSED_DISTANCE and state in [State.DOOR_PASSED, State.INIT]) or (state == State.DOOR_FOUND and min(self._select_lasers('front', 'distance')) <= CLOSED_DISTANCE):
                    state = State.DOOR_SEARCH

                elif state == State.DOOR_FOUND and min(self._select_lasers('front', 'distance')) > CLOSED_DISTANCE and not True in [self.flappy_on_the_move['up'], self.flappy_on_the_move['down']]:
                    state = State.GO_THROUGH_DOOR

                elif state == State.PASSING_IN_DOOR and min(self._select_lasers('front', 'distance')) < CLOSED_DISTANCE :
                    state = State.DOOR_PASSED

                elif state in [State.GO_THROUGH_DOOR, State.PASSING_IN_DOOR]:
                    state = State.PASSING_IN_DOOR

                # COMMAND REGARDING STATE

                if state == State.DOOR_SEARCH:
                    self.search_free_path()

                elif state == State.DOOR_FOUND:
                    self.stop_y_flappy_vel()

                elif state == State.GO_THROUGH_DOOR:
                    self.move_flappy_forward()

                elif state == State.PASSING_IN_DOOR:
                    self.command_acc_x == 0.0

                elif state == State.DOOR_PASSED:
                    # self.stop_x_flappy_vel()
                    self.obstacles_crossed_number += 1
                    rospy.loginfo(f'New obstacle crossed [number = {self.obstacles_crossed_number}] !')

                if min(self._select_lasers('front', 'distance')) < FRONT_LETHAL_DISTANCE:
                    self.stop_x_flappy_vel()

                if state != old_state:

                    rospy.loginfo(f'>>> STATE = {state}')
                    rospy.loginfo(f'Acc command => for x = {self.command_acc_x} && for y = {self.command_acc_y}')
                    old_state = state

                # ACC COMMAND

                self._pub_acc_cmd.publish(
                    Vector3(
                        self.command_acc_x,
                        self.command_acc_y,
                        0.0
                    )
                )

            except rospy.ROSInterruptException:
                break

    def _select_lasers (self, location, data_type):
        """
        [Select and fetch the values of the data_type chosen for the lasers in the location wanted]

        Args:
            location ([str]): ['front', 'bottom', 'top']
            data_type ([str]): ['distance', 'contact']

        Returns:
            [list]: [list of the data_type laser value for the location chosen]
        """        

        assert location in ['front', 'bottom', 'top']
        assert data_type in ['distance', 'contact']

        if location == 'front':
            i_list = [3, 4, 5]

        elif location == 'bottom':
            i_list = [0]

        elif location == 'top':
            i_list = [8]

        returned_list = list()

        for laser in self.lidar_dict.values():

            if laser['number'] in i_list:
                returned_list.append(laser[data_type])

        return returned_list

    def _vel_callback (self, vel_vector):
        """
        [Callback of /flappy_vel topic => Allows to fetch vel and stabalize it if to high]

        Args:
            vel_vector ([Vector3]): [topic /flappy_vel value]
        """        

        self.current_vel = vel_vector

        if self.game_is_started:
        
            # VEL X STABILISATION

            if self.current_vel.x >= self.command_acc_x:

                rospy.logdebug(f'X Speed reached for {self.current_vel.x}m/s.')
                self.command_acc_x = 0.0

            # TICK Y STABILISATION

            if fabs(self.current_vel.y) >= fabs(self.command_acc_y):

                if not self.tick_y_reached: # STOP Y ACC

                    rospy.logdebug(f'Y tick reached.')
                    self.stop_y_flappy_vel()
                    self.tick_y_reached = True

            else:
                self.tick_y_reached = False

            # ON THE MOVE DEFINITION

            if fabs(self.current_vel.x) < 0.005 and self.flappy_on_the_move['forward']:

                self.flappy_on_the_move['forward'] = False
                self.command_acc_x = 0.0

            if fabs(self.current_vel.y) < 0.005 and (self.flappy_on_the_move['up'] or self.flappy_on_the_move['down']):

                self.flappy_on_the_move['up'] = False
                self.flappy_on_the_move['down'] = False
                self.command_acc_y = 0.0

    def _laser_scan_callback (self, msg):
        """
        [Callback of /flappy_laser_scan topic => Allows to fetch all lasers data and to store it inside one dict]

        Args:
            msg ([LaserScan]): [topic /flappy_laser_scan value]
        """        

        if not self.game_is_started:

            self.game_is_started = True
            self.time_init = time.time()

        i = 0

        for (laser_measurement, laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['distance'] = laser_measurement
            self.lidar_dict[f'laser_{i}']['number'] = i
            self.lidar_dict[f'laser_{i}']['contact'] = bool(int(laser_intensity))
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            i += 1

        assert self.lidar_dict['laser_8']['angle'] == msg.angle_max, f"Issue regarding angle increment ! Last laser angle [{self.lidar_dict[f'laser_8']['angle']}] != angle_max [{msg.angle_max}]"

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

    def search_free_path (self):
        """
        [Via the distance of each laser, define a lateral direction to perform]
        """        

        free_path_detected_list = []

        for laser in self.lidar_dict.values():

            if not laser['contact'] or laser['distance'] > CLOSED_DISTANCE:

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

            self.stop_y_flappy_vel()

            if 4 in free_path_detected_list:

                if 5 in free_path_detected_list:
                    self.move_flappy_up()

                elif 3 in free_path_detected_list:
                    self.move_flappy_down()

######################################################################################################################################################

if __name__ == '__main__':

    try:

        flappy_automation_node = FlappyAutomationNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################