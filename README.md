## install ubuntu 22.04 // ros2 humble hawksbill
...




## initialize the workspace
```
mkdir -p ~/turtle_n_fly_ws/src/
cd ~/turtle_n_fly_ws/src/
git clone https://github.com/uunotap/turtle_n_fly.git
```



## Install Webots
```
sudo apt-get install webots
sudo apt-get install ros-humble-webots-ros2
```

## Add Webots to the ws (? Might be redundant...)
```
cd ~/turtle_n_fly_ws/src/
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git
cd ~/turtle_n_fly_ws/
colcon build --symlink-install
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






