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

## Troubleshooting
```
 pip install numpy==1.24.4
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






