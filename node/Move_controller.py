#!/usr/bin/env python3
import rospy
import yaml
from pathlib import Path
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import argparse
import numpy as np
import os


red_carState_log = np.zeros((5000, 2), dtype = object)
red_carState_counter = 0
blue_carState_log = np.zeros((5000, 2), dtype = object)
blue_carState_counter = 0
lidar_filename = ""

# # LiDAR Logging
# red_lidar_log = np.zeros((5000, 1082))
# red_lidar_counter = 0
# blue_lidar_log = np.zeros((5000, 1082))
# blue_lidar_counter = 0

moves_started = False



def parse_moves_list(moves_filename):
    moves_filepath = Path(os.getcwd() + "/" + moves_filename)
    moves_list = []
    with moves_filepath.open('r') as moves_file:
        yaml_loader = yaml.safe_load_all(moves_file)
        for m in yaml_loader:
            moves_list.append(m)
            
    return moves_list
    
def publish_car_msg(car_topic, car_msg, move):
    car_msg.drive.steering_angle = move['steering_angle'] 
    car_msg.drive.speed = move['speed']
    car_msg.header.stamp = rospy.Time.now()
    car_topic.publish(car_msg)
    
def red_carState_logger(msg):
    global moves_started
    if moves_started:
        global red_carState_log
        global red_carState_counter
        red_carState_log[red_carState_counter,0] = rospy.Time.now().to_sec()
        red_carState_log[ red_carState_counter,1] = msg
        red_carState_counter += 1
    
def blue_carState_logger(msg):
    global moves_started
    if moves_started:
        global blue_carState_log
        global blue_carState_counter
        blue_carState_log[blue_carState_counter, 0] = rospy.Time.now().to_sec()
        blue_carState_log[blue_carState_counter, 1] = msg
        blue_carState_counter += 1

# LiDAR Logging
# def red_lidar_logger(msg):
#     global moves_started
#     if moves_started:    
#         global red_lidar_log 
#         global red_lidar_counter
#         red_lidar_log[red_lidar_counter, 0] = rospy.Time.now().to_sec()
#         red_lidar_log[red_lidar_counter, 1:] = np.asarray(msg.ranges).flatten()
#         red_lidar_counter += 1

# def blue_lidar_logger(msg):
#     global moves_started
#     if moves_started:
#         global blue_lidar_log 
#         global blue_lidar_counter
#         blue_lidar_log[blue_lidar_counter, 0] = rospy.Time.now().to_sec()
#         blue_lidar_log[blue_lidar_counter, 1:] = np.asarray(msg.ranges).flatten()
#         blue_lidar_counter += 1
    
def run_movesets(red_moves, blue_moves):
   
    # Topics for obtaining car info for logging
    
    # blue_carState_topic = rospy.get_param("~carState_topic_blue")
    rospy.Subscriber("/carState_topic_blue", String, blue_carState_logger)
    # red_carState_topic = rospy.get_param("~carState_topic_red")
    rospy.Subscriber("/carState_topic_red", String, red_carState_logger)
    
    
    # Topics for obtaining LiDAR info for Collisions Testing
    # # scan_topic_blue = rospy.get_param("~scan_topic_blue")
    # rospy.Subscriber("/blue/scan", LaserScan, blue_lidar_logger)
    # # scan_topic_red = rospy.get_param("~scan_topic_red")
    # rospy.Subscriber("/red/scan", LaserScan, red_lidar_logger)
    

    # Publsiihers for the drive commands
    # blue_drive_topic = rospy.get_param("~drive_topic_blue")
    blue_pub = rospy.Publisher("/drive_blue", AckermannDriveStamped, queue_size=200)
    blue_msg = AckermannDriveStamped()
    
    # red_drive_topic = rospy.get_param("~drive_topic_red")
    red_pub = rospy.Publisher("/drive_red", AckermannDriveStamped, queue_size=200)
    red_msg = AckermannDriveStamped()
            
    
  
    # Variabes used to track moves for each car
    finished = False
    curr_red_move = 0
    curr_blue_move = 0
    blue_move = blue_moves[curr_blue_move]
    red_move = red_moves[curr_red_move]
    curr_time = rospy.Time.now()
    blue_time = curr_time + rospy.Duration(blue_move['time'])
    red_time = curr_time + rospy.Duration(red_move['time'])
    global moves_started
    moves_started = True
    publish_car_msg(blue_pub, blue_msg, blue_move)
    publish_car_msg(red_pub, red_msg, red_move)
    start_time = rospy.Time.now()
    rospy.loginfo(f"Starting Movesets")
    
    # Running the moves
    while (not(finished)):
        curr_time = rospy.Time.now()
        if curr_red_move == len(red_moves) and curr_blue_move == len(blue_moves):
            print ("All moves complete")
            finished = True
            break
        if curr_time >= red_time:
            if curr_red_move < len(red_moves) - 1:
                curr_red_move += 1
                red_move = red_moves[curr_red_move]
                red_time = curr_time + rospy.Duration(red_move['time'])
                
                print (f"Red move complete @ {rospy.Time.now()}")
                print (f"new angle: {red_move['steering_angle']}")
                
            elif curr_red_move == len(red_moves) - 1:
                curr_red_move += 1
        print (f"publishing angle {red_move['steering_angle']}")
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
                           
        
        

    
    
    # Stopping the cars
    blue_msg.drive.steering_angle = 0.0
    blue_msg.drive.speed = 0.0        
    red_msg.drive.steering_angle = 0.0
    red_msg.drive.speed = 0.0
    blue_pub.publish(blue_msg)
    red_pub.publish(red_msg)  
    moves_started = False
    time_taken = rospy.Time.now() - start_time
    rospy.loginfo(f"Movesets Complete. Time taken: {time_taken.to_sec()}")
    
    return time_taken

def get_test_dirname(test):
    test_count = 1
    
    new_test_dir = Path("{}/{}/run_{}".format(os.getcwd(), test, test_count))
    while os.path.exists(new_test_dir):
        test_count += 1
        new_test_dir = Path("{}/{}/run_{}".format(os.getcwd(), test, test_count))
        
    return new_test_dir

if __name__ == '__main__':
    
    # Parsing args for movesets and test dir
    parser = argparse.ArgumentParser()
    parser.add_argument('--bluemoves', '-bm', type=str, required=True, help='Moveset YAML file to be loaded for blue car')    
    parser.add_argument('--redmoves', '-rm', type=str, required=True, help='Moveset YAML file to be loaded for red car')    
    parser.add_argument('--outputdir', '-od', type=str, required=False, help='Directory to save test data for all runs')
    args = parser.parse_args()
    
    # Getting movesets for cars
    blue_moves = parse_moves_list(args.bluemoves)
    red_moves = parse_moves_list(args.redmoves)

    
    
    
    # Starts a new node
    rospy.init_node('Move_controller', anonymous=True)
    
    # Publishing rate of 100Hz
    rospy.Rate(100)
    
    # Running YAML Movesets
    try:
        time_taken = run_movesets(red_moves, blue_moves)
    except rospy.ROSInterruptException:
        pass
    rospy.sleep(1)
    rospy.signal_shutdown("Moves Complete: Closing Node")
    
    if args.outputdir is None:
        exit()
        
    save_data = input("Save data? (y/n): ")
   
    while save_data.lower() != "y":
        if save_data.lower() == "n":
            exit()
        else:
            save_data = input("Invalid input. Save data? (y/n): ")
   
    # Obtaining dir for logging test data
    test_dir = get_test_dirname(args.outputdir)
    os.makedirs(test_dir, exist_ok=False)
    
    print("Saving Car State Data to {}".format(test_dir))
    
    red_carState_file = "{}/red_carState_log.txt".format(test_dir)
    blue_carState_file = "{}/blue_carState_log.txt".format(test_dir)
    red_carState_log = red_carState_log[np.all(red_carState_log != 0, axis=1)]
    blue_carState_log = blue_carState_log[np.all(blue_carState_log != 0, axis=1)]

    np.savetxt(red_carState_file, red_carState_log, fmt='%s', delimiter=",")
    np.savetxt(blue_carState_file, blue_carState_log, fmt='%s', delimiter=",")
    
    # print ("Saving LiDAR Data")
    # red_lidar_file = "{}/red_lidar_log.txt".format(test_dir)
    # blue_lidar_file = "{}/blue_lidar_log.txt".format(test_dir)
    # red_lidar_log = red_lidar_log[np.all(red_lidar_log != 0, axis=1)]
    # np.savetxt(red_lidar_file, red_lidar_log, fmt='%s', delimiter=",")
    # blue_lidar_log = blue_lidar_log[np.all(blue_lidar_log != 0, axis=1)]
    # np.savetxt(blue_lidar_file, blue_lidar_log, fmt='%s', delimiter=",")
    
    terminal_file = "{}/terminal_output.txt".format(test_dir)
    with open(terminal_file, "w") as f:
        f.write("Elapsed Time: {}\n\n".format(time_taken.to_sec()))
    print (len(blue_carState_log))
    print (len(red_carState_log))
    print ("Closing Move Controller")
    exit()