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
    
class Manoeuvre_metrics:
    def __init__(self, number):
        self.number = number
        self.red_xs = np.array([])
        self.red_ys = np.array([])
        self.blue_xs = np.array([])
        self.blue_ys =  np.array([])
        self.angles = np.array([])
        self.manoeuvre_times = np.array([])
    
    def print_metrics(self):
        print (f"Manouevre Metrics: for Manouevre {self.number}")
        print ("Red xs: ", self.red_xs)
        print ("Red ys: ", self.red_ys)
        print ("Blue xs: ", self.blue_xs)
        print ("Blue ys: ", self.blue_ys)
        print ("Angles: ", self.angles)
        print ("Manoeuvre Times: ", self.manoeuvre_times)
        print ("")
    
    def print_mean_metrics(self):
        print(f"Mean Metrics for Manoeuvre {self.number}")
        print(f"Mean Red Position: {np.mean(self.red_xs)}, {np.mean(self.red_ys)}")
        print(f"Mean Blue Position: {np.mean(self.blue_xs)}, {np.mean(self.blue_ys)}")
        print(f"Mean Angle: {np.mean(self.angles)}")
        print(f"Mean Maneuvre Time: {np.mean(self.manoeuvre_times)}")
        print()
        
        
    def read_data(self, f):
        coords = re.split('[:,=\n]', f.readline())
        self.red_xs = np.append(self.red_xs, float(coords[2]))
        self.red_ys = np.append(self.red_ys, float(coords[4]))
        
        coords = re.split('[:,=\n]', f.readline())
        self.blue_xs = np.append(self.blue_xs, float(coords[2]))
        self.blue_ys = np.append(self.blue_ys, float(coords[4]))

        self.angles = np.append(self.angles, float(f.readline().split(':')[1]))
        self.manoeuvre_times = np.append(self.manoeuvre_times, float(f.readline().split(':')[1]))        
        
    def save_means(self, f):
        f.write(f"Mean Metrics for Manoeuvre {self.number}\n")
        f.write(f"Mean Red Position: {np.mean(self.red_xs)}, {np.mean(self.red_ys)}\n")
        f.write(f"Mean Blue Position: {np.mean(self.blue_xs)}, {np.mean(self.blue_ys)}\n")
        f.write(f"Mean Angle: {np.mean(self.angles)}\n")
        f.write(f"Mean Maneuvre Time: {np.mean(self.manoeuvre_times)}\n\n")
    
violation_count = 0
manoeuvre_count = -1
time_count = 3
sum_time = 0

parser = argparse.ArgumentParser()
parser.add_argument('--file', '-f', type=str, required=True, help='Test Folder for Calculating Results')   
parser.add_argument('--run_number', '-r', type=str, required=False, help='Number of test runs')    
args = parser.parse_args()

num_runs = 10
if args.run_number:
    num_runs = int(args.run_number)
    
manoeuvre_1 = Manoeuvre_metrics(1)
manoeuvre_2 = Manoeuvre_metrics(2)
manoeuvres = np.array([manoeuvre_1, manoeuvre_2])

for i in range(1, 11):
    file_path = Path(os.path.join(os.path.dirname(__file__), args.file, f"run_{i}", "terminal_output.txt"))
    print (f"Reading file: {file_path}")
    
    with file_path.open('r') as f:
        first_line = f.readline() 
        if "Elapsed Time" in first_line:
            sum_time += float(first_line.split(":")[-1])
        else:
            raise Exception(f"Time not reported in this file")
        
        for line in f:
            if "Defensive Move" in line:
                break
                manoeuvre_count += 1
        
        manoeuvre_1.read_data(f)   
             
        f.readline()
        f.readline()
        f.readline()
        manoeuvre_2.read_data(f)
        f.readline()
        f.readline()
        if "VIOLATION" in f.readline():
            violation_count += 1
         
    # break        
    


manoeuvre_1.print_mean_metrics()
manoeuvre_2.print_mean_metrics()

with Path(os.path.join(os.path.dirname(__file__), args.file, "average_metrics.txt")).open('w') as f:
    manoeuvre_1.save_means(f)
    manoeuvre_2.save_means(f)
    f.write(f"Mean Number of Violations: {violation_count/violation_count}\n")
    f.write(f"Mean Time of Run: {sum_time/num_runs}\n")