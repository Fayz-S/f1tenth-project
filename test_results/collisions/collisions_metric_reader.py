import os
import sys
import argparse
from pathlib import Path
import numpy as np
import re

def get_user_input(metric_name):
    var = input ("Would you like to analyse the " + metric_name + " metric? (y/n): ")
    while var.lower() != "y":
        if var.lower() == "n":
            return False
        else:
            var = input ("Invalid input. Please enter 'y' or 'n': ")
    
    
red_lidars = np.array([])
blue_lidars = np.array([])
red_xs = np.array([])
blue_xs = np.array([])
red_ys = np.array([])
blue_ys = np.array([])
car_distances = np.array([])



parser = argparse.ArgumentParser()
parser.add_argument('--file', '-f', type=str, required=True, help='Test Folder for Calculating Results')   
parser.add_argument('--run_number', '-r', type=str, required=False, help='Number of test runs')    
args = parser.parse_args()

num_runs = 10
if args.run_number:
    num_runs = int(args.run_number)
    

lidar_desired = get_user_input("LiDAR")
car_distance_desired = get_user_input("Car Distance") 
violation_count = 0

for i in range(1, 11):
    file_path = Path(os.path.join(os.path.dirname(__file__), args.file, f"run_{i}", "terminal_output.txt"))
    print (f"Reading file: {file_path}")
    with file_path.open('r') as f:
        for line in f:z
            if "Red Min LiDAR" in line:
                #print(line)
                red_lidars = np.append(red_lidars, float(line.split(": ")[1]))
            elif "Blue Min LiDAR" in line:
                #print(line)
                blue_lidars = np.append(blue_lidars, float(line.split(": ")[1]))
            elif "Red: x" in line:
                #print(line)
                coords = re.split('[:,=\n]', line)
                # print (coords) 
                red_xs = np.append(red_xs, float(coords[2]))
                red_ys = np.append(red_ys, float(coords[4]))
            elif "Blue: x" in line:
                #print(line)
                coords = re.split('[:,=\n]', line)
                blue_xs = np.append(blue_xs, float(coords[2]))
                blue_ys = np.append(blue_ys, float(coords[4]))
            elif "Car Distance" in line:
                #print(line)
                car_distances = np.append(car_distances, float(line.split(": ")[1]))
            elif "VIOLATION" in line:
                violation_count += 1
                
if len(red_xs) > num_runs:
    raise Exception("More than one value logged per run for red x")
elif len(red_xs) < num_runs:
    raise Exception("Red x not logged for every run")

if len(blue_xs) > num_runs:
    raise Exception("More than one value logged per run for blue x")
elif len(blue_xs) < num_runs:
    raise Exception("Blue x not logged for every run")

if len(red_ys) > num_runs:
    raise Exception("More than one value logged per run for red y")
elif len(red_ys) < num_runs:
    raise Exception("Red y not logged for every run")

if len(blue_ys) > num_runs:
    raise Exception("More than one value logged per run for blue y")
elif len(blue_ys) < num_runs:
    raise Exception("Blue y not logged for every run")

if lidar_desired:
    if len(red_lidars) > num_runs:
        raise Exception("More than one value logged per run for red lidar")
    elif len(red_lidars) < num_runs:
        raise Exception("Red LiDAR not logged for every run")

    if len(blue_lidars) > num_runs:
        raise Exception("More than one value logged per run for blue lidar")
    elif len(blue_lidars) < num_runs:
        raise Exception("Blue LiDAR not logged for every run")

if car_distance_desired:
    if len(car_distances) > num_runs:
        raise Exception("More than one value logged per run for car distance")
    elif len(car_distances) < num_runs:
        raise Exception("Car distance not logged for every run")



# Logging Mean of Metrics+
print(f"Mean Blue Position: {np.mean(blue_xs)}, {np.mean(blue_ys)}")
print(f"Mean Red Position: {np.mean(red_xs)}, {np.mean(red_ys)}")
print(f"Mean Blue LiDAR: {np.mean(blue_lidars)}") if lidar_desired else None
print(f"Mean Red LiDAR: {np.mean(red_lidars)}") if lidar_desired else None
print(f"Mean Car Distance: {np.mean(car_distances)}") if car_distance_desired else None
print (f"Mean Number of Violations: {violation_count/num_runs}")

with Path(os.path.join(os.path.dirname(__file__), args.file, "average_metrics.txt")).open('w') as f:
    f.write(f"Mean Blue Position: {np.mean(blue_xs)}, {np.mean(blue_ys)}\n")
    f.write(f"Mean Red Position: {np.mean(red_xs)}, {np.mean(red_ys)}\n")
    f.write(f"Mean Blue LiDAR: {np.mean(blue_lidars)}\n") if lidar_desired else None
    f.write(f"Mean Red LiDAR: {np.mean(red_lidars)}\n") if lidar_desired else None
    f.write(f"Mean Car Distance: {np.mean(car_distances)}\n") if car_distance_desired else None
    f.write(f"Mean Number of Violations: {violation_count/num_runs}")
# Logging Standard Deviation of Metrics
# print (f"Standard Deviation of Red Position: {np.std(red_xs)}, {np.std(red_ys)}")
# print (f"Standard Deviation of Blue Position: {np.std(blue_xs)}, {np.std(blue_ys)}")
# print (f"Standard Deviation of Red LiDAR: {np.std(red_lidars)}")
# print (f"Standard Deviation of Blue LiDAR: {np.std(blue_lidars)}")
# print (f"Standard Deviation of Car Distance: {np.std(car_distances)}")

            
    