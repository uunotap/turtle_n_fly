# Project info



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






