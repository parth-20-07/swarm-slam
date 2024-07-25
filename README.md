Build:

```bash
catkin build && source ~/catkin_ws/devel/setup.zsh
```

Launch Gazebo
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```