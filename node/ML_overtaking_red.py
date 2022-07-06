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

from tensorflow.python.keras.models import load_model

"""
This node subscribes red car's LiDAR and carState, publish driving command for red car
"""

# how many timestamps will be used?
# larger input means the model will use more inputs data, but the performance will not necessarily be better!
size_of_input = 5
LiDAR_raw_array = Queue(maxsize=size_of_input)
car_state_array = Queue(maxsize=size_of_input)


def LiDAR_callback(data):
    global LiDAR_raw_array

    # Note the ros message is not related to programming language, same type of message can be read in both Python and C
    LiDAR_raw = data.ranges

    LiDAR_raw = np.asarray(LiDAR_raw)
    LiDAR_raw = np.reshape(LiDAR_raw, (1,-1))

    # First In First Out
    if LiDAR_raw_array.full():
        LiDAR_raw_array.get()
    LiDAR_raw_array.put(LiDAR_raw)


def carState_callback(data):
    global car_state_array

    car_state = np.asarray(data.data.split(","), dtype=float)

    speed_steering = []
    # car_state[3] is Velocity_X
    speed_steering.append(car_state[3])
    # car_state[5] is Steering_angle
    speed_steering.append(car_state[5])
    speed_steering = np.asarray(speed_steering)
    speed_steering = np.reshape(speed_steering, (1,-1))

    if car_state_array.full():
        car_state_array.get()
    car_state_array.put(speed_steering)


if __name__ == '__main__':

    rospy.init_node("ML_overtake_red", anonymous=True)
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
    rospack = rospkg.RosPack()
    # structure of LSTM model is same as RNN_12_3
    overtaking_model = load_model(rospack.get_path("f1tenth_simulator_two_agents")+'/overtaking_models/model_RNN_12_3')
    overtaking_model.summary()
    overtaking_model.compile(loss="mean_absolute_error", optimizer="adam", metrics=['mean_absolute_error'])

    carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber(carState_topic, String, carState_callback)

    scan_topic_red = rospy.get_param("~scan_topic_red")
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)

    overtaking_drive_topic = rospy.get_param("~overtaking_drive_topic")
    drive_pub_red = rospy.Publisher(overtaking_drive_topic, AckermannDriveStamped, queue_size=10)

    # the LiDAR_raw_array need at least one LiDAR data in it
    rospy.wait_for_message(scan_topic_red, LaserScan)

    while not rospy.is_shutdown():
        inputA = np.asarray(list(LiDAR_raw_array.queue))
        inputB = np.asarray(list(car_state_array.queue))

        # inputA: (size_of_input, 1, 1081) means there are size_of_input number of instances, each instance is 1*1081
        # inputB: (size_of_input, 1, 2) means there are size_of_input number of instances, each instance is 1*2
        lengthA = inputA.shape[0]
        lengthB = inputB.shape[0]
        length_min = min(lengthA, lengthB)

        # After both LiDAR_raw_array and car_state_array reached size_of_input, we can start feed them to the model
        if length_min == size_of_input:

            # we want there are just one instance, each instance is size_of_input*1081 or 2
            inputA = np.reshape(inputA, (1, size_of_input, -1))
            inputB = np.reshape(inputB, (1, size_of_input, -1))
            # rospy.loginfo(inputA.shape)
            # rospy.loginfo(inputB.shape)
            # inputA: (1, size_of_input, 1081)
            # inputB: (1, size_of_input, 2)

            # if you want to see more info, change verbose to 1
            command = overtaking_model.predict([inputA, inputB], verbose=0)

            # rospy.loginfo(command)
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.drive.steering_angle = command[0,-1,1]*0.24
            ack_msg.drive.speed = command[0,-1,0]*16
            drive_pub_red.publish(ack_msg)
