import os
import sys
import argparse
from pathlib import Path
import numpy as np
import re

class Car_pass_metrics:
    def __init__(self, name):
        self.car_name = name
        self.start_times = np.array([])
        self.apex_times = np.array([])
        self.end_times = np.array([])
    
    def read_first_pass(self, f):
        next(f)
        start_time = float(next(f).split(":")[-1]) 
        self.start_times = np.append(self.start_times, 0)
        return start_time
    
    def read_pass(self, f, first_line, start_time):
        next(f)
        time = float(next(f).split(":")[-1]) - start_time
        
        if "apex" in first_line:
            self.apex_times = np.append(self.apex_times, time)
        elif "end" in first_line:
            self.end_times = np.append(self.end_times, time)
        elif "start" in first_line:
            self.start_times = np.append(self.start_times, time)
        else:
            raise Exception("Unknown car pass type")
        next(f)
    
    def print_metrics(self, start_time = 0):
        print (f"Metrics for {self.car_name} car")
        print (f"Start Pass: {self.start_times + start_time}")
        print (f"Apex Pass: {self.apex_times + start_time}")
        print (f"End Pass: {self.end_times + start_time}")
        print ()   
    
    def print_mean_metrics(self):
        print (f"Metrics for {self.car_name} car")
        print (f"Start Pass: {np.mean(self.start_times)}")
        print (f"Apex Pass: {np.mean(self.apex_times)}")
        print (f"End Pass: {np.mean(self.end_times)}")
        print ()
    
    def save_metrics(self, f):
        f.write(f"Metrics for {self.car_name} car\n")
        f.write(f"Mean time for passing start: {np.mean(self.start_times)}\n")
        f.write(f"Mean time for passing apex: {np.mean(self.apex_times)}\n")
        f.write(f"Mean time for passing end: {np.mean(self.end_times)}\n\n")
        
 
parser = argparse.ArgumentParser()
parser.add_argument('--file', '-f', type=str, required=True, help='Test Folder for Calculating Results')   
parser.add_argument('--run_number', '-r', type=str, required=False, help='Number of test runs')    
args = parser.parse_args()



num_runs = 10
if args.run_number:
    num_runs = int(args.run_number)
    
blue_metrics = Car_pass_metrics("Blue")
red_metrics = Car_pass_metrics("Red")
violation_count = 0
sum_time = 0

for i in range(1, 11):
    file_path = Path(os.path.join(os.path.dirname(__file__), args.file, f"run_{i}", "terminal_output.txt"))
    print (f"Reading file: {file_path}")
    blue_count = 0
    red_count = 0
    with file_path.open('r') as f:
        first_line = f.readline()
        if "Elapsed Time" in first_line:
            sum_time += float(first_line.split(":")[-1])
        else:
            raise Exception(f"Time not reported in this file")
            
        for line in f:
            if "Creating 1 swatches" in line:
                line = next(f)
                break
        
        if "Red car" in line:
            start_time = red_metrics.read_first_pass(f)
            red_count += 1
        elif "Blue car" in line:
            start_time = blue_metrics.read_first_pass(f)
            blue_count += 1
        else:
            raise Exception(f"This line should not appear here, parsing error")   
        next(f)
        
        for line in f:
            # print (line)
            if "Red car has" in line:
                red_metrics.read_pass(f, line, start_time)
                red_count += 1
            elif "Blue car has" in line:
                blue_metrics.read_pass(f, line, start_time)
                blue_count += 1
            elif "VIOLATION" in line:
                violation_count += 1
                break
            elif red_count == 3 and blue_count == 3:
                break
            else:
                raise Exception(f"This line should not appear here, parsing error")   
          

with Path(os.path.join(os.path.dirname(__file__), args.file, "average_metrics.txt")).open('w') as f:
    blue_metrics.save_metrics(f)
    red_metrics.save_metrics(f)
    if violation_count == 0:
        f.write(f"Mean Number of Violations: {0}")
    else:
        f.write(f"Mean Number of Violations: {violation_count/violation_count}\n")
    
    f.write (f"Mean Time of Run: {sum_time/num_runs}\n")