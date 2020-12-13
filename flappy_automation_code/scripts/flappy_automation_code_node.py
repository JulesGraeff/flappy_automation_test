#!/usr/bin/env python

#####################################################################################################################################################

# @project        IROS Partiel
# @file           path/to/file
# @author         Jules Graeff

######################################################################################################################################################

import rospy
from math import fabs
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

######################################################################################################################################################

CLOSED_DISTANCE = 2.5
LETHAL_DISTANCE = 0.6
VEL_INCREMENT = 0.25

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
        self.game_is_started = False
        self.front_lasers_measurement_list = [CLOSED_DISTANCE]
        self.bottom_lasers_measurement_list = [CLOSED_DISTANCE]
        self.top_lasers_measurement_list = [CLOSED_DISTANCE]

        self._up_counter = 0
        self._down_counter = 0

        self.flappy_on_the_move = dict(forward = False, up = False, down = False)
        
        self.command_acc_y = 0.0
        self.command_acc_x = 0.0
        self.vel_x_reached = False
        self.vel_y_reached = False

        self.trying_to_find_path = False

        while not self.game_is_started:
            pass

        self.move_flappy_forward()

        while not rospy.is_shutdown():

            try:

                if self.lidar_dict['laser_4']['dist_measured'] < LETHAL_DISTANCE or self.lidar_dict['laser_5']['dist_measured'] < LETHAL_DISTANCE or self.lidar_dict['laser_3']['dist_measured'] < LETHAL_DISTANCE:

                    # rospy.logwarn(rospy.get_caller_id() + f' An obstacle has been detected aroun Flappy !!')

                    if self.lidar_dict['laser_4']['dist_measured'] < LETHAL_DISTANCE or (self.lidar_dict['laser_3']['dist_measured'] < LETHAL_DISTANCE and self.lidar_dict['laser_5']['dist_measured'] < LETHAL_DISTANCE):
                        
                        if self.flappy_on_the_move['forward']:
                            self.stop_flappy()

                    if self.lidar_dict['laser_4']['dist_measured'] > LETHAL_DISTANCE:

                        if min(self.bottom_lasers_measurement_list) <= LETHAL_DISTANCE and not self.flappy_on_the_move['up']:
                            self.move_flappy_up(VEL_INCREMENT)

                        elif min(self.top_lasers_measurement_list) <= LETHAL_DISTANCE and not self.flappy_on_the_move['down']:
                            self.move_flappy_down(VEL_INCREMENT)

                        if min(self.bottom_lasers_measurement_list) <= LETHAL_DISTANCE or min(self.top_lasers_measurement_list) <= LETHAL_DISTANCE and not self.flappy_on_the_move['forward']:
                            self.move_flappy_forward(VEL_INCREMENT)
                
                elif not self.flappy_on_the_move['forward']:
                    self.move_flappy_forward()

                lateral_obstacle = min(self.bottom_lasers_measurement_list) <= LETHAL_DISTANCE and min(self.top_lasers_measurement_list) <= LETHAL_DISTANCE and min(self.front_lasers_measurement_list) > LETHAL_DISTANCE

                if min(self.front_lasers_measurement_list) < CLOSED_DISTANCE and not lateral_obstacle:
                    
                    self.find_free_path()
                    self.trying_to_find_path = True

            except rospy.ROSInterruptException:
                break
        

    def _vel_callback (self, msg):

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on self.current_vel.x velCallback

        self.current_vel =  msg
        
        if self.current_vel.x >= self.command_acc_x:

            if not self.vel_x_reached: # STOP ACC

                print(f'X Speed reached for {self.current_vel.x}m/s.')
                self._pub_acc_cmd.publish( Vector3(0.0, 0.0, 0.0) )
                self.vel_x_reached = True

        else:
            self.vel_x_reached = False

        if self.trying_to_find_path:

            if fabs(self.current_vel.y) >= fabs(self.command_acc_y):

                if not self.vel_y_reached: # STOP ACC

                    print(f'Y Speed reached for {self.current_vel.x} m/s.')
                    self._pub_acc_cmd.publish( Vector3(0.0, 0.0, 0.0) )
                    self.vel_y_reached = True

            else:
                self.vel_y_reached = False

        if fabs(self.current_vel.x) < 0.001:
            self.flappy_on_the_move['forward'] = False

        if fabs(self.current_vel.y) < 0.001:
            self.flappy_on_the_move['up'] = False
            self.flappy_on_the_move['down'] = False

    def _laser_scan_callback (self, msg):

        if not self.game_is_started:

            self.game_is_started = True
            self.time_init = time.time()

        i = 0

        self.front_lasers_measurement_list.clear()
        self.top_lasers_measurement_list.clear()
        self.bottom_lasers_measurement_list.clear()

        for (laser_measurment, laser_intensity) in zip(msg.ranges, msg.intensities):

            self.lidar_dict[f'laser_{i}'] = dict()
            self.lidar_dict[f'laser_{i}']['dist_measured'] = laser_measurment
            self.lidar_dict[f'laser_{i}']['number'] = i
            self.lidar_dict[f'laser_{i}']['intensity'] = laser_intensity
            self.lidar_dict[f'laser_{i}']['angle'] = msg.angle_min + (msg.angle_increment * i)

            if i in [0, 1]:
                self.bottom_lasers_measurement_list.append(laser_measurment)
            
            if i in [7, 8]:
                self.top_lasers_measurement_list.append(laser_measurment)

            if i in [3, 4, 5]:
                self.front_lasers_measurement_list.append(laser_measurment)

            i+=1

        assert self.lidar_dict['laser_8']['angle'] == msg.angle_max, f"Issue regarding angle increment ! Last laser angle [{self.lidar_dict[f'laser_8']['angle']}] != angle_max [{msg.angle_max}]"

        if min(self.top_lasers_measurement_list) <= LETHAL_DISTANCE:
            rospy.logwarn('TOP LASER ARE INFORMING OF A COLLISION!')

        if min(self.bottom_lasers_measurement_list) <= LETHAL_DISTANCE:
            rospy.logwarn('DOWN LASER ARE INFORMING OF A COLLISION!')

        if min(self.front_lasers_measurement_list) <= LETHAL_DISTANCE:
            rospy.logwarn('FRONT LASER ARE INFORMING OF A COLLISION!')

    def stop_flappy (self):

        print(f'STOP => self.current_vel.x = {self.current_vel.x}')

        self._pub_acc_cmd.publish(
            Vector3(
                -self.current_vel.x*2,
                -self.current_vel.y*2,
                -self.current_vel.z*2,
            )
        )

    def move_flappy_forward (self, acc=VEL_INCREMENT):

        self.command_acc_x = acc

        print(f'MOVE FORWARD => self.command_acc_x = {self.command_acc_x} ')

        self._pub_acc_cmd.publish(
            Vector3(
                self.command_acc_x,
                0.0,
                0.0
            )
        )
        self.flappy_on_the_move['forward'] = True

    def move_flappy_up (self, vel=(VEL_INCREMENT/10.0)):

        self.command_acc_y = vel

        self._pub_acc_cmd.publish(
            Vector3(
                0.0,
                self.command_acc_y,
                0.0
            )
        )
        self.flappy_on_the_move['up'] = True
        self.flappy_on_the_move['down'] = False

    def move_flappy_down (self, vel=(VEL_INCREMENT/10.0)):

        self.command_acc_y = -vel

        self._pub_acc_cmd.publish(
            Vector3(
                0.0,
                self.command_acc_y,
                0.0
            )
        )
        self.flappy_on_the_move['down'] = True
        self.flappy_on_the_move['up'] = False

    def find_free_path (self):

        # list_laser 
        
        # if True in [laser['intensity'] == 0.0 in self.lidar_dict.values()]:

        no_obstacle_detected_list = []

        for laser in self.lidar_dict.values():

            if laser['intensity'] == 0.0 or laser['dist_measured'] > CLOSED_DISTANCE:
                # print(f"Laser number {laser['number']} is not encountring any cloosed obstacle")

                no_obstacle_detected_list.append(laser['number'])

        free_laser_number_mean = 4.0

        if len(no_obstacle_detected_list) != 0:
            free_laser_number_mean = sum(no_obstacle_detected_list) / len(no_obstacle_detected_list)

        # print(f'free_laser_number_mean = {free_laser_number_mean} ')

        if free_laser_number_mean > 4.0:

            self._up_counter += 1

            if not self.flappy_on_the_move['up'] or self._up_counter>50000:

                print('UP')

                self.move_flappy_up()
                # self.move_flappy_up((free_laser_number_mean - 4.0) / 10.0)
                self._up_counter = 0
            
            # self.move_flappy_up(free_laser_number_mean - 4.0)

        elif free_laser_number_mean < 4.0:

            self._down_counter += 1

            if not self.flappy_on_the_move['down'] or self._down_counter>50000:

                print('DOWN')

                self.move_flappy_down()
                # self.move_flappy_down((free_laser_number_mean - 4.0) / 10.0)
                self._down_counter = 0

            # self.move_flappy_up(free_laser_number_mean - 4.0)

        else:
            # print('STAND BY')
            pass

######################################################################################################################################################

if __name__ == '__main__':

    try:

        flappy_automation_node = FlappyAutomationNode()

        # Ros spin to prevent program from exiting
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

######################################################################################################################################################