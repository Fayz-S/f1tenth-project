#!/usr/bin/env python3

from queue import Queue
import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
"""
This node subscribes red car and blue cars LiDAR and carState to detect 
rule violations with regard to collisions and overtaking on straights.
"""

# --- Class for storing state of car informtaion --- #
class CarState:
    def __init__(self, msg):
        self.x = msg[0]
        self.y = msg[1]
        self.theta = msg[2]    
        self.velocity_x = msg[3]
        self.velocity_y = msg[4]
        self.steering_angle = msg[5]
        self.angular_velocity = msg[6]
        self.slip_angle = msg[7]
        
# --- Global variables (used by Callback Functions) --- #
red_car_state = CarState([0,0,0,0,0,0,0,0])
blue_car_state = CarState([0,0,0,0,0,0,0,0])
red_lidar_data = np.zeros(1081)
blue_lidar_data = np.zeros(1081)

# --- Variables for calculating lidar distance --- #
lidar_min_distances = np.array([])


# --- Callbalck functions --- #
def msg_to_carState(msg):
    carState_array = np.asarray(msg.data.split(","), dtype=float)
    return CarState(carState_array)

def red_carState_callback(data):
    global red_car_state
    red_car_state = msg_to_carState(data)

def blue_carState_callback(data):
    global blue_car_state
    blue_car_state = msg_to_carState(data)
    

def red_lidar_callback(data):
    global red_lidar_data
    red_lidar_data = np.asarray(data.ranges).flatten()

def blue_lidar_callback(data):
    global blue_lidar_data
    blue_lidar_data = np.asarray(data.ranges).flatten()

# --- Functions for Checking Corner 1 --- #
def entered_corner_1(x):
    if x < 14.77:
        return True
    else:
        return False

def crossed_apex_1(car_x, car_y):
    m = 1.318423129
    b = 14.07025964
    y = m * car_x + b
    if car_y >= y:
        return True
    else:
        return False

def exited_corner_1(y):
    if y > 33.96:
        return True   
    else:
        return False 

# --- Functions for Checking Corner 2 --- #
def entered_corner_2(x):
    if x > 31.063597:
        return True
    else:
        return False

def crossed_apex_2(car_x, car_y):
    # m = -1.173206956
    # b = 74.14200188
    m = -0.9798153087
    b = 67.99347353
    y = m * car_x + b
    if car_y >= y:
        return True
    else:
        return False

def exited_corner_2(y):
    if y > 38.476059:
        return True   
    else:
        return False
  
  
      
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
    red_carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber(red_carState_topic, String, red_carState_callback)
    
    blue_carState_topic = rospy.get_param("~carState_topic_blue")
    rospy.Subscriber(blue_carState_topic, String, blue_carState_callback)
    
    scan_topic_red = rospy.get_param("~scan_topic_red")
    rospy.Subscriber(scan_topic_red, LaserScan, red_lidar_callback)
    
    scan_topic_blue = rospy.get_param("~scan_topic_blue")
    rospy.Subscriber(scan_topic_blue, LaserScan, blue_lidar_callback)
    
    # Variables for collision detection
    # min_lidar_val = 1
    collision_reported = False    
    # Variables for defensive manoeuvre
    min_turing_angle = 1
    on_straight = True
    angle_changed_recent = False
    timer = rospy.Duration(0)
    steering_threshold = 0.1
    defence_move_time = rospy.Duration(2)
    angle = 0
    

    # Variables for overtaking on corners
    
    red_passed_apex = False
    red_passed_start = False
    red_passed_end = False
    
    blue_passed_apex = False
    blue_passed_start = False
    blue_passed_end = False
    
    car_entered_corner = False
    car_passed_apex = False
    car_exited_corner = False
    
    overtake_illegal = False
    red_ahead_at_start = False
    not_reported = True
    corner_violation = False
    c = 0
    rospy.sleep(1)
    red_lidar_cumul = np.zeros(1081)
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        # print (np.sort(red_lidar_data)[0:10])
        # np.savetxt("/home/fayzs/Documents/diss/f1tenth/src/f1tenth-project/red_lidar_data", red_lidar_data, delimiter=",")
        # input("Press Enter to rerun.")
        # print("Waiting 5 seconds...")
        # red_lidar_cumul += red_lidar_data
        # c += 1
        # if c == 1000:
        #     print(np.sort(red_lidar_cumul/1000)[0:10])
        #     c = 0
        #     red_lidar_cumul = np.zeros(1081)
        #     rospy.sleep(5)
        
        
        # -------------------- Extra Functionality -------------------- #
        
        # --- Calculating lidar threshold --- #
        # Mean value calculated for red behind blue was 0.014686912528741231
        # Mean value calculated for red beside blue to was 0.22484801780581476

        # Mean value calculated for red in front of blue was 0.6662225331068039
        # run 2: 0.22361359398663044
    
        # if red_lidar_data[0] != 0:
        #     # np.savetxt("/home/fayzs/Documents/diss/f1tenth/src/f1tenth-project/lidar_output/max_distance_collision", lidar_data, delimiter=",")
        #     # np.savetxt("/home/fayzs/Documents/diss/f1tenth/src/f1tenth-project/lidar_output/max_distance_collision_sorted", sorted(lidar_data), delimiter=",")
        #     while not rospy.is_shutdown() and len(lidar_min_distances) < 10000:
        #         min_lidar_value = np.min(red_lidar_data)
        #         lidar_min_distances = np.append(lidar_min_distances, min_lidar_value)
        #         rospy.sleep(0.01)
        #     if len(lidar_min_distances) == 10000:
        #         # print("Completed LiDAR Distance Analysis")  
        #         print(f"Average Min Distance Value: {lidar_min_distances.mean()}")
        #         lidar_min_distances = np.append(lidar_min_distances, lidar_min_distances.mean())
        #         np.savetxt("/home/fayzs/Documents/diss/f1tenth/src/f1tenth-project/lidar_output/run_2/side_test_1", lidar_min_distances, delimiter=",")

        #         input("Press Enter to rerun.")
        #         # rospy.sleep(3)
        #         lidar_min_distances = np.array([])
        
        # --- Calculating Turning Threshold  --- #
        
        # if rospy.get_time() % .5 < 0.00005:
        #     print(blue_car_state.steering_angle)
        
        # --- Detecting if car has passed different points on curve --- #
        # if rospy.get_time() % .5 < 0.00005:
        #     print(blue_car_state.x, blue_car_state.y)
       
        # --- Detecting if car has passed different points on curve --- #
        # if rospy.get_time() % .5 < 0.00005:
        #     print(blue_car_state.x, blue_car_state.y)
    
    # -------------------- Main Implementation -------------------- #    
    
    # --- Collision Implementation --- #
        car_distance = np.sqrt((blue_car_state.x - red_car_state.x)**2 + (blue_car_state.y - red_car_state.y)**2)
        if car_distance <= 0.52:
            red_min_lidar = np.min(red_lidar_data)
            blue_min_lidar = np.min(blue_lidar_data)
            if (red_min_lidar < 0.22561 or blue_min_lidar < 0.22561):# and not(collision_reported):
                print (f"Red Min LiDAR: {red_min_lidar}")
                print (f"Blue Min LiDAR: {blue_min_lidar}")
                print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
                print(f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
                print(f"Car Distance: {car_distance}")
                print ("VIOLATION: LiDAR Detected Collision between cars")
                break
                collision_reported = True
                
    # # --- Defensive Manoeuvre Implementation --- #    
        # if on_straight:
        #     if (red_car_state.steering_angle > steering_threshold or 
        #         red_car_state.steering_angle < -steering_threshold):# and (red_car_state.velocity_x > 0.2 or red_car_state.velocity_y > 0.2):
        #         if angle_changed_recent:
        #             if rospy.Time.now() - timer < defence_move_time:
        #                 if (abs(angle - red_car_state.steering_angle) > steering_threshold * 2 and 
        #                     abs(angle + red_car_state.steering_angle) < steering_threshold / 2):
                            
        #                     print ("Defensive Move Attempted by Red Car on Straight")
        #                     print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
        #                     print(f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
        #                     print (f"Red Steering Angle Triggering Manoeuvre: {angle}")
        #                     print (f"Manoeuvre Time: {(rospy.Time.now() - timer).to_sec()}")
        #                     print (f"Time: {rospy.Time.now().to_sec() - start_time}\n")
                                
        #                     if defensive_move_made:
        #                         print ("VIOLATION: Red car attempted defensive maneouvre more than once on straight")
        #                         # print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
        #                         # print(f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
                                
        #                         print (f"Time: {rospy.Time.now().to_sec() - start_time}\n")
        #                         break
                            
        #                     else:
        #                         defensive_move_made = True
        #                         angle_changed_recent = False
        #             else:
        #                 angle_changed_recent = False
        #         else:
        #             angle_changed_recent = True
        #             angle = red_car_state.steering_angle
        #             timer = rospy.Time.now()

    # --- Overtaking on corners Implementation --- #

        if not(blue_passed_start) and entered_corner_2(blue_car_state.x):
            print ("Blue car has passed start of corner")
            print (f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            blue_passed_start = True
            
        elif not(blue_passed_end) and exited_corner_2(blue_car_state.y):
            print ("Blue car has passed end of corner")
            print (f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            blue_passed_end = True
        
        elif not(blue_passed_apex) and crossed_apex_2(blue_car_state.x, blue_car_state.y):
            print ("Blue car has passed apex of corner")
            print (f"Blue: x = {blue_car_state.x}, y = {blue_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            blue_passed_apex = True
            
        if not(red_passed_start) and entered_corner_2(red_car_state.x):
            print ("Red car has passed start of corner")
            print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            red_passed_start = True
        
        elif not(red_passed_end) and exited_corner_2(red_car_state.y):
            print ("Red car has passed end of corner")
            print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            red_passed_end = True
        
        elif not(red_passed_apex) and crossed_apex_2(red_car_state.x, red_car_state.y):
            print ("Red car has passed apex of corner")
            print (f"Red: x = {red_car_state.x}, y = {red_car_state.y}")
            print (f"Time of Pass: {rospy.Time.now().to_sec() - start_time}\n")
            red_passed_apex = True
        
        if not(car_entered_corner):
            if entered_corner_2(red_car_state.x):
                car_entered_corner = True
                red_ahead_at_start = True
            elif entered_corner_2(blue_car_state.x):
                car_entered_corner = True
                
            # -> Implementation without using above boolean variables 
        if (car_entered_corner and not(car_exited_corner)):
            if not(car_passed_apex):
                if crossed_apex_2(red_car_state.x, red_car_state.y):
                    car_passed_apex = True
                elif crossed_apex_2(blue_car_state.x, blue_car_state.y):
                    car_passed_apex = True
                    if not(red_ahead_at_start):
                        overtake_illegal = True   
            
            if exited_corner_2(red_car_state.y):
                if not(exited_corner_2(blue_car_state.y)) and overtake_illegal:
                    corner_violation = True
                    car_exited_corner = True
                    not_reported = False
            
            elif (exited_corner_2(blue_car_state.y)):
                car_exited_corner = True 
                
                
        if corner_violation and red_passed_end and blue_passed_end:
            print ("VIOLATION: Red car attempted illegal overtake on corner")
            corner_violation = False
            
            #  -> Implementation using above boolean variables +
        # if not(car_entered_corner):
        #     if red_passed_start:
        #         car_entered_corner = True
        #         red_ahead_at_start = True
        #     elif blue_passed_start:
        #         car_entered_corner = True

        # if not(car_passed_apex):
        #     if red_passed_apex:
        #         car_passed_apex = True
        #     elif blue_passed_apex:
        #         car_passed_apex = True
        #         if not(red_ahead_at_start):
                    
        #             overtake_illegal = True

        # if red_passed_end and not(blue_passed_end) and overtake_illegal:
        #     corner_violation = True
        #     not_reported = False
            
        # if corner_violation and red_passed_end and blue_passed_end:
        #     print ("VIOLATION: Red car attempted illegal overtake on corner")
        #     corner_violation = False
        
                

    # --- Saving files from extra functionality section --- #

        
        
