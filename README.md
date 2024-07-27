Build:

```bash
catkin build && source ~/catkin_ws/devel/setup.zsh
```

Launch Gazebo
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch SLAM Node
```bash
roslaunch slam slam_launch.launch
```

Launch Teleop for Turtlebot
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```


## Multiple Robot Situation

Teleop
```bash
roslaunch slam random_movement.launch
```

Gazebo
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```