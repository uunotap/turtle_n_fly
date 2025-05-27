## install ubuntu 22.04 // ros2 humble hawksbill
...




## initialize the workspace
mkdir -r ~/turtle_n_fly/src/
cd ~/turtle_n_fly/src/
git clone *thispackage*
...

## install TurtleBot 4 Common's (edited from the turtlebot4 common documentation)
cd ~/turtle_n_fly/src/
git clone https://github.com/turtlebot/turtlebot4.git -b humble
cd ..
rosdep install --from-path src -yi --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install

## install turtlebot4 simulation package to the workspace (edited from the turtlebot4 simulation documentation)
cd ~/turtle_n_fly/src/
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
cd ..
rosdep install --from-path src -yi --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install


## ??? aerial drone ??? 
...



### source the workspace
source ~/turtle_n_fly_ws/install/setup.bash
### Launch the turtlebot4 test environment
ros2 launch turtlebot4_ignition_bringup ignition.launch.py 
### Launch the turtlebot under namespace "tb4"
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb4 use_sim_time:=true
### Launching the aerial drone



