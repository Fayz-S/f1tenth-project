# [DISSERTATION] F1TENTH Simulator Two-Agents


> This repository is the first part of my dissertation "[F1TENTH](https://f1tenth.org/index.html): Development of A Multi-Agent Simulator and An Overtaking Algorithm Using Machine Learning".  
> This is a lightweight 2D simulator of [F1TENTH Racecar](https://f1tenth.org/index.html). This simulator has 2D-LiDAR simulation, collision checker, vehicle model, planning algorithm, and other features.

Please feel free to raise topics in Issues section, I will try my best to answer them!


## Environment


### Software


- Operating System: Ubuntu 20.04 (amd64/arm64)
- Robot Operating System: ROS Noetic (desktop-full)     
    
If you want to use overtaking algorithm:   
> [Depends on your GPU](https://www.tensorflow.org/install/pip#hardware_requirements), there are two different ways of installing tensorflow. If your GPU has cuda cores, you can install from pip. If not, you must build from source. Moreover, you can install tensorflow and other python packages in a Miniconda env, but you must activate that environment before launch the simulator in a new terminal every time.
- tensorflow 2.10.0 (I think 2.9.0/2.8.0/2.7.0 should also work)
- numpy 1.22.4

If you want to use MPC algorithm:

- casadi 3.5.5
- numpy 1.22.4
- pandas 1.4.2


### Hardware (recommended)


> If you are using virtual machine, better using a virtual machine that support GPU virtualization, or you can dual-boot. I am using Parallels VM and developed everything in this virtual machine, no performance issue with M1 MAX.

- dual-boot  
CPU: 4 cores with 3GHz  
GPU: GTX 1660  
RAM: 8GB

- virtual machine   
CPU: 8 cores    
GPU: RTX 2080   
RAM: 16GB   

## Install for first time

If you have ```ros-noetic-desktop-full``` installed, the additional dependencies you must install are:

- tf2_geometry_msgs
- ackermann_msgs
- joy
- map_server

You can install them by running:

    sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server


To install the simulator package, clone the repository with the simulator and starter code into your catkin workspace by running command below:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/JZ76/f1tenth_simulator_two_agents.git
    
Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make


## Every time open the simulator in a new terminal


To run the simulator in a new terminal, run:

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch f1tenth_simulator_two_agents simulator.launch

> Note: after run `cd ~/catkin_ws` and `source devel/setup.bash` commands, you can keep that terminal opening, so that you don't have to run these two commands before launch the simulator every time.


## Every time you edit some code in C++ files


To make your changes works, you need to build the simulator again, by using `catkin_make`:

    catkin_make

**Summary: Once you sourced a terminal, there are only two commands used frequently, one is `catkin_make` when you did some changes, another is `roslaunch f1tenth_simulator simulator.launch` to open the simulator**


## Uninstall

    cd ~/catkin_ws/src
    rm -rf f1tenth_simulator_two_agents
    cd ..
    catkin_make
    
You also need to manually delete `f1tenth_simulator_two_agents` dictionary in `~/catkin_ws/build`


## Usage


Please see [Wiki section](https://github.com/JZ76/f1tenth_simulator_two_agents/wiki) for instruction in detail


### RVIZ Visualization

With the simulator running, open rviz.
In the left panel at the bottom click the "Add" button, then in the "By topic" tab add the ```/map``` topic and the ```/scan``` topic.
Then in the "By display type" tab add the RobotModel type.
In the left panel under the newly added LaserScan section, change the size to 0.1 meters for a clearer visualization of the lidar (shown in rainbow).

You can use a keyboard or USB joystick to drive the car around, or you can place the car manually by clicking the "2D Pose Estimate button" on the top of the screen and dragging your mouse on the desired pose.

### ROS API

The simulator was set up with two main objectives in mind- similitude to the real car and fast prototyping of racing algorithms. The *simulator* node was written such that it can be swapped out with the F1/10 car itself, and if all topic names remain the same, the same exact code can be run to drive the car. The rest of the ROS nodes are organized so that new planning algorithms can be added quickly and toggled between during driving.

![Simplified graph of ROS nodes](https://github.com/f1tenth/f1tenth_simulator/blob/master/media/sim_graph_public.png)

Our public simulator includes a simple *random driver* node as an example for what a planning node should look like. Each planner can listen to the sensor data published by the *simulator* and then publish [AckermannDrive](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to their own specific topic (e.g., ```/random_drive```). The *mux* node listens to all of these topics, then takes the message from whichever planner is turned on and publishes it to the main ```/drive``` topic, which the *simulator* listens to. Note that only the velocity and steering angle specified in the message are used. The *mux* node also listens to joystick and keyboard messages too, for manual driving.
The *behavior controller* node tells the *mux* node which planner is on through the ```/mux``` topic. By default, each planner (including keyboard and joystick) is mapped to a joystick button and keyboard key, and they are simply toggled on and off manually. 
Additionally, upon collision, the car will halt and all mux channels will be clear- nothing will be in control until manual intervention.

To instantly move the car to a new state publish [Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) messages to the ```/pose``` topic. This can be useful for scripting the car through a series of automated tests.

The simulated lidar is published to the ```/scan``` topic as [LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html) messages.

The pose of the car is broadcast as a transformation between the ```map``` frame and the ```base_link``` frame. ```base_link``` is the center of the rear axis. The ```laser``` frame defines the frame from which the lidar scan is taken and another transform is broadcast between it and ```base_link```.

### What you can do

If you plan to change the behavior of the car beyond keyboard, joystick, or direct pose control, you will mostly be writing new code in new planning nodes and the *behavior controller* node. Steps for adding a new planner are detailed below. By default, the *behavior controller* listens to sensor messages, so you could write the controller such that the car switches autonomously between planners during a race depending on these dynamic inputs.

### Adding a planning node

There are several steps that necessary to adding a new planning node. There is commented out code in each place that details exactly what to do. Here are the steps:

* Make a new node that publishes to a new drive topic- look at *random_walker* for an example
* Launch the node in the launch file ```simulator.launch```
* Make a new ```Channel``` instance at the end of the Mux() constructor in ```mux.cpp```
* Add if statement to the end of the joystick and keyboard callbacks (key\_callback(), joy\_callback) in ```behavior_controller.cpp```

In ```params.yaml```, add the following:

* a new drive topic name
* a new mux index
* a new keyboard character (must be a single alphabet letter)
* a new joystick button index

You'll need to get the mux index and drive topic name in ```mux.cpp``` for the new ```Channel```, and the keyboard character, mux index, and joystick button index will all need to be added as member variables in ```behavior_controller.cpp```. Your planning node will obviously need the drive topic name as well.



