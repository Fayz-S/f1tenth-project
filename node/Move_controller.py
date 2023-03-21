#!/usr/bin/env python3
import rospy
import yaml
from pathlib import Path
from ackermann_msgs.msg import AckermannDriveStamped
import std_msgs.msg
import argparse
import time
import numpy as np
import os

def parse_moves_list(moves_filename):
    moves_filepath = Path(os.getcwd() + "/moves/" + moves_filename)
    moves_list = []
    with moves_filepath.open('r') as f:
        yaml_loader = yaml.safe_load_all(f)
        for m in yaml_loader:
            moves_list.append(m)
            
    return moves_list
    
def publish_car_msg(car_topic, car_msg, move):
    car_msg.drive.steering_angle = move['steering_angle'] 
    car_msg.drive.speed = move['speed']
    print(f"speed published: {move['speed']}")
    car_msg.header.stamp = rospy.Time.now()
    car_topic.publish(car_msg)
    
def move(args):

    print ("Starting Move")
    
    # drive_topic_blue = rospy.get_param("~drive_topic_blue")
    # drive_topic_red  = rospy.get_param("/drive_red")
    
    red_pub = rospy.Publisher('/drive_red', AckermannDriveStamped, queue_size=10)
    red_msg = AckermannDriveStamped()
    red_moves = parse_moves_list(args.redmoves)

            
    blue_pub = rospy.Publisher('/drive_blue', AckermannDriveStamped, queue_size=10)
    blue_msg = AckermannDriveStamped()
    blue_moves = parse_moves_list(args.bluemoves)
  
    finished = False
    curr_red_move = 0
    curr_blue_move = 0
    red_move = red_moves[curr_red_move]
    blue_move = blue_moves[curr_blue_move]
    curr_time = rospy.Time.now()
    red_time = curr_time + rospy.Duration(red_move['time'])
    blue_time = curr_time + rospy.Duration(blue_move['time'])
    publish_car_msg(red_pub, red_msg, red_move)
    publish_car_msg(blue_pub, blue_msg, blue_move)
    
    
    while (not(finished)):
        curr_time = rospy.Time.now()
        if curr_red_move == len(red_moves) and curr_blue_move == len(blue_moves):
            print ("All moves complete")
            finished = True
        
        if curr_time >= red_time:
            if curr_red_move < len(red_moves) - 1:
                curr_red_move += 1
                red_move = red_moves[curr_red_move]
                red_time = curr_time + rospy.Duration(red_move['time'])
                
                print (f"Red move complete @ {rospy.Time.now()}")
                print (f"new angle: {red_move['steering_angle']}")
                
            elif curr_red_move == len(red_moves) - 1:
                curr_red_move += 1
        publish_car_msg(red_pub, red_msg, red_move)
        if curr_time>= blue_time:
            if curr_blue_move < len(blue_moves) - 1:
                curr_blue_move += 1
                blue_move = blue_moves[curr_blue_move]
                blue_time = curr_time + rospy.Duration(blue_move['time'])
                print ("Blue move complete")
                

            elif curr_blue_move == len(blue_moves) - 1:
                curr_blue_move += 1
        publish_car_msg(blue_pub, blue_msg, blue_move)                      
        
        
        # publish_car_msg(red_pub, red_msg, red_move)
        # publish_car_msg(blue_pub, blue_msg, blue_move)
      
    blue_msg.drive.steering_angle = 0.0
    blue_msg.drive.speed = 0.0        
    red_msg.drive.steering_angle = 0.0
    red_msg.drive.speed = 0.0
    blue_pub.publish(blue_msg)
    red_pub.publish(red_msg)  
        
        
if __name__ == '__main__':
    print((os.getcwd()))
    print(os.path.dirname(os.getcwd()))
    # Starts a new node
    rospy.init_node('Move_controller', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('--redmoves', '-rm', type=str, required=True, help='Name of the YAML file to be loaded for red car')    
    parser.add_argument('--bluemoves', '-bm', type=str, required=False, help='Name of the YAML file to be loaded for blue car')    
    args = parser.parse_args()
    print (args.redmoves)
    try:
        # Testing our function
        move(args)
    except rospy.ROSInterruptException:
        pass
    print ("Moves complete")
    exit()