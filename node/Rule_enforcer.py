#!/usr/bin/env python3

"""
Copyright 2022 Jiancheng Zhang

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from queue import Queue
import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
"""
This node subscribes red car's LiDAR and carState, publish driving command for red car
"""

# how many timestamps will be used?
# larger input means the model will use more inputs data, but the performance will not necessarily be better!
size_of_input = 6

class CarState:
    def __init__(self, msg):
        self.x = msg[0]
        self.y = msg[1]
        self.theta = msg[2]    
        self.velocity_x = msg[3]
        self.velocity_y = msg[4]
        self.steer_angle = msg[5]
        self.angular_velocity = msg[6]
        self.slip_angle = msg[7]
        
red_car_state = CarState([0,0,0,0,0,0,0,0])


    
    
def msg_to_carState(msg):
    carState_array = np.asarray(msg.data.split(","), dtype=float)
    return CarState(carState_array)

def carState_callback(data):
    global red_car_state
    red_car_state = msg_to_carState(data)

if __name__ == '__main__':

    rospy.init_node("Rule_enforcer", anonymous=True)
    """
    Australia 5 7 8 9 10 11 12 13 14
    Shanghai 9 10 12 14
    14_5 14_3 12_3 10_4 10_3

    14 = none -> Australia
    14_1 = 14 -> Shanghai
    14_2 = 14_1 -> Gulf data
    14_5 = 14_2 -> all data
    14_3 = 14_2 -> malaysian data

    12 = none -> Australia
    12_1 = 12 -> Shanghai
    12_2 = 12_1 -> Gulf data
    12_3 = 12_2 -> malaysian data

    10 = none -> Australia
    10_1 = 10 -> Shanghai
    10_2 = 10_1 -> Gulf data
    10_3 = 10_2 -> malaysian data
    10_4 = 10_3 -> Australia+malaysian
    """ 
    timer = 0
    defensive_move_made = False
    carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber(carState_topic, String, carState_callback)
    on_straight = True
    angle_changed = False
    while not rospy.is_shutdown():
        # if (red_car_state.steer_angle > 0.1 or red_car_state.steer_angle < -0.1) and red_car_state.velocity_x > 0.5:
        #     print("red car is turning")
        #     if float(rospy.get_time() - timer) < 0.25:
        #         print ("Red car attempted defensive maneouvre")
        #         if not defensive_move:
        #             defensive_move = True
        #             timer = rospy.get_time()
        #     else:
        #         timer = rospy.get_time()
        # if (rospy.get_time() % 1 < 0.00000005):
        #     print(red_car_state.steer_angle)
        if on_straight:
            
            if red_car_state.steer_angle > 0.1 or red_car_state.steer_angle < -0.1:
                if angle_changed:
                    
            # if (red_car_state.angular_velocity > 0.1 or red_car_state.angular_velocity < -0.1) and red_car_state.velocity_x > 0.5:
            #     # print("red car is turning")
            #     if float(rospy.get_time() - timer) < 1 and float(rospy.get_time() - timer) > 0.5:
                    
            #         if defensive_move_made:
            #             print ("VIOLATION: Red car attempted defensive maneouvre more than once on straight")
            #         else:        
            #             defensive_move_made = True
            #     timer = rospy.get_time()
            
        # print (red_car_state.x, red_car_state.y) 
        # if (rospy.get_time() % 5 < 0.00000005):
        #     print(red_car_state.steer_angle)
