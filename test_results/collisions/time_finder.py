import os
import sys
import argparse
from pathlib import Path
import numpy as np
import re

parser = argparse.ArgumentParser()
parser.add_argument('--file', '-f', type=str, required=True, help='Test Folder for Calculating Results')   
parser.add_argument('--run_number', '-r', type=str, required=False, help='Number of test runs')    
args = parser.parse_args()
runtimes = np.array([])
test_types = ["ahead", "behind", "left", "right"]
logs = ["blue_carState_log.txt", "red_carState_log.txt", "blue_lidar_log.txt", "red_lidar_log.txt"]

for test in test_types:
    for log in logs:
        
        for i in range(1, 11):
            file_path = Path(os.path.join(os.path.dirname(__file__), args.file, f"run_{i}", log))
            print (f"Reading file: {file_path}")
            with file_path.open('r') as f:
                first_time = f.readline().split(",")[0]
                for line in f:
                    last_time = line.split(",")[0]

            runtimes = np.append(runtimes, float(last_time) - float(first_time))

print (runtimes.mean())
  
 
            
    
      


            
    