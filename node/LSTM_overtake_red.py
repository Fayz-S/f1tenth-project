#!/usr/bin/env python3
from tensorflow.python.keras.models import load_model
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from queue import Queue

size_of_input = 5
LiDAR_raw_array = Queue(maxsize=size_of_input)
car_state_array = Queue(maxsize=size_of_input)


def LiDAR_callback(data):
    global LiDAR_raw_array
    LiDAR_raw = data.ranges
    LiDAR_raw = np.asarray(LiDAR_raw)
    LiDAR_raw = np.reshape(LiDAR_raw, (1,-1))
    if LiDAR_raw_array.full():
        LiDAR_raw_array.get()
    LiDAR_raw_array.put(LiDAR_raw)

def carState_callback(data):
    global car_state_array
    car_state = np.asarray(data.data.split(","), dtype=float)
    speed_steering = []
    speed_steering.append(car_state[3])
    speed_steering.append(car_state[5])
    speed_steering = np.asarray(speed_steering)
    speed_steering = np.reshape(speed_steering, (1,-1))
    if car_state_array.full():
        car_state_array.get()
    car_state_array.put(speed_steering)

if __name__ == '__main__':
    rospy.init_node("LSTM_overtake_red", anonymous=True)
    # Australia 5 7 8 9 10 11 12 13 14
    # Shanghai 9 10 12 14
    # 14_5 14_3 12_3 10_4 10_3

    # 14 = none -> Australia
    # 14_1 = 14 -> Shanghai
    # 14_2 = 14_1 -> Gulf data
    # 14_5 = 14_2 -> all data
    # 14_3 = 14_2 -> malaysian data

    # 12 = none -> Australia
    # 12_1 = 12 -> Shanghai
    # 12_2 = 12_1 -> Gulf data
    # 12_3 = 12_2 -> malaysian data

    # 10 = none -> Australia
    # 10_1 = 10 -> Shanghai
    # 10_2 = 10_1 -> Gulf data
    # 10_3 = 10_2 -> malaysian data
    # 10_4 = 10_3 -> Australia+malaysian

    # structure of LSTM model is same as RNN_12_3
    LSTM_model = load_model('/home/jz76/catkin_ws/src/f1tenth_simulator/model_RNN_14_5')
    LSTM_model.summary()
    LSTM_model.compile(loss="mean_absolute_error", optimizer="adam", metrics=['mean_absolute_error'])

    LSTM_drive_topic = rospy.get_param("~LSTM_drive_topic")
    scan_topic_red = rospy.get_param("~scan_topic_red")
    carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber(carState_topic, String, carState_callback)
    drive_pub_red = rospy.Publisher(LSTM_drive_topic, AckermannDriveStamped, queue_size=10)
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)

    rospy.wait_for_message(scan_topic_red, LaserScan)

    while not rospy.is_shutdown():
        inputA = np.asarray(list(LiDAR_raw_array.queue))
        inputB = np.asarray(list(car_state_array.queue))
        # LiDAR_raw_array = []
        # car_state_array = []

        lengthA = inputA.shape[0]
        lengthB = inputB.shape[0]

        length_min = min(lengthA, lengthB)
        if length_min == size_of_input:

            inputA = np.reshape(inputA, (1, size_of_input, -1))
            inputB = np.reshape(inputB, (1, size_of_input, -1))
            # rospy.loginfo(inputA.shape)
            # rospy.loginfo(inputB.shape)
            command = LSTM_model.predict([inputA, inputB], verbose=0)

            # rospy.loginfo(command)
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.drive.steering_angle = command[0,-1,1]*0.192
            ack_msg.drive.speed = command[0,-1,0]*14
            drive_pub_red.publish(ack_msg)
