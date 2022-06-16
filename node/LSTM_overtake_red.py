#!/usr/bin/env python3
from keras.models import load_model
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import math

LiDAR_raw_array = []
car_state_array = []

def LiDAR_callback(data):
    global LiDAR_raw_array
    LiDAR_raw = data.ranges
    LiDAR_raw = np.asarray(LiDAR_raw)
    LiDAR_raw = np.reshape(LiDAR_raw, (1,-1))
    LiDAR_raw_array.append(LiDAR_raw)

def carState_callback(data):
    global car_state_array
    car_state = np.asarray(data.data.split(","), dtype=float)
    car_state[2] = math.fmod(car_state[2] , 2*math.pi)
    speed_steering = []
    speed_steering.append(car_state[3])
    speed_steering.append(car_state[5])
    speed_steering = np.asarray(speed_steering)
    speed_steering = np.reshape(speed_steering, (1,-1))
    car_state_array.append(speed_steering)

if __name__ == '__main__':
    rospy.init_node("LSTM_overtake_red", anonymous=True)

    LSTM_model = load_model('/home/jz76/catkin_ws/src/f1tenth_simulator/model_RNN_10')
    LSTM_model.summary()
    LSTM_model.compile(loss="mean_absolute_error", optimizer="adam", metrics=['mean_absolute_error'])

    LSTM_drive_topic = rospy.get_param("~LSTM_drive_topic")
    scan_topic_red = rospy.get_param("~scan_topic_red")
    carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber(carState_topic, String, carState_callback)
    rospy.loginfo("started")
    drive_pub_red = rospy.Publisher(LSTM_drive_topic, AckermannDriveStamped, queue_size=10)
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)

    rospy.wait_for_message(scan_topic_red, LaserScan)

    while not rospy.is_shutdown():
        inputA = np.asarray(LiDAR_raw_array)
        inputB = np.asarray(car_state_array)
        LiDAR_raw_array = []
        car_state_array = []
        lengthA = len(inputA)
        lengthB = len(inputB)
        length_min = min(lengthA, lengthB)
        if length_min is not 0:
            command = LSTM_model.predict([inputA[:length_min], inputB[:length_min]], verbose=0)
            # rospy.loginfo(command)
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.drive.steering_angle = command[-1,0,1]*0.192
            ack_msg.drive.speed = command[-1,0,0]*16
            drive_pub_red.publish(ack_msg)
