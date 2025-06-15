# Project info

# Introduction
This project is part of the AR MRS 2025 course and demonstrates a collaborative multi-robot system in a simulated environment using ROS 2 and Webots. A TurtleBot3 scouts a terrain to identify a safe landing zone, then communicates the coordinates to a Mavic 2 Pro drone, which autonomously navigates to the spot and lands.

The simulation is entirely run in Webots R2025a and leverages the ROS 2 Humble Hawksbill middleware. This repository contains all the necessary code, launch files, and setup instructions to reproduce the system.

Follow the instructions below to set up your environment and run the simulation.


# General outline of the project files
```
├── launch 
│   ├── empty.launch.py     #Start test world with only the robots
│   ├── mavic_test.launch.py #Start test world with only the drone
│   ├── turtle_n_fly.launch.py #Start testing world with some obstacles and both robots
│   └── turtle_test.launch.py #Start test world with only the turtle
├── README.md
├── resource
│   ├── mavic_webots.urdf # Drone definition
│   ├── nav2_params.yaml # Turtlebot simulation parameters
│   ├── ros2control.yml # Turtlebot simulation parameters
│   ├── turtlebot_webots.urdf # Turtlebot definition
│   └── turtle_n_fly # default file
├── setup.py #Marking files for colcon
├── test # default files
│   ├── test_copyright.py
│   ├── test_flake8.py 
│   └── test_pep257.py # default file
├── turtle_n_fly
│   ├── drone_controller.py # Logic for the drone
│   ├── __init__.py # default file
│   ├── mavic_driver.py # Slightly modified cyberbotics(webots) example driver
│   └── turtle_node.py # Logic for the turtle
├── worlds
│   ├── empty.wbt # def for empty world with both
│   ├── mavic_empty.wbt # def for empty with drone
│   ├── testworld.wbt # def for the test
│   └── turtle_empty.wbt # def for empty with turtle
└─ ...
```





# RUNNING/REPRODUCIBILITY
## install ubuntu 22.04 // ros2 humble hawksbill
```
https://docs.ros.org/en/humble/Installation.html
```

## initialize the workspace
```
mkdir -p ~/turtle_n_fly_ws/src/
cd ~/turtle_n_fly_ws/src/
git clone https://github.com/uunotap/turtle_n_fly.git
```

## Add Webots (webots_ros2 R2025a) to the ws 
```
cd ~/turtle_n_fly_ws/src/
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git
cd ~/turtle_n_fly_ws/
colcon build #somewhat important since alot of the webots things have cmake, that doesn't like being rebuild
```

## Additional Python requirements
```
 pip install numpy==1.24.4
 pip install opencv-python
```


## .bashrc utility
```
source /opt/ros/humble/setup.bash
export WEBOTS_HOME=/home/ut/.ros/webotsR2025a/webots 
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH

```

### Running the simulation
```
cd ~/turtle_n_fly_ws/
colcon build --symlink-install
source install/setup.bash
ros2 launch turtle_n_fly turtle_n_fly.launch.py
```


## Findings and future possibilities
We noticed that the teamworks of two robots was feasible in the simulation environment. The speed of the turtlebot finding the suitable landing area, and the smooth landing of the drone can be improved upon, but they work fine for a proof of consept. Although the communication between the drones is limited to mostly the TurtleBot sending location info to the drone, it could be improved upon so that the turtlebot could use the drones vantage point to possible find the optimal route faster.



