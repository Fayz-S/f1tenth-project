import matplotlib.pyplot as plt
import numpy as np
import pandas as pd 
import os
file_path = os.path.dirname(os.path.realpath(__file__)) + "/lidar_output/min_collision_distances_side"
lidar_data = np.loadtxt (file_path)
x = np.arange(1, len(lidar_data) + 1)    
# Plot the circle
plt.plot(x, lidar_data, 'ro')

# Show the plot
plt.show()
