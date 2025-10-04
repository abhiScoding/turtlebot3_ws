
- launch gazebo simulation
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

- display odometry message
$ ros2 run my_pkg odom_subscriber

- publish goal Point for the bot to move
$ ros2 run my_pkg go_to_goal
$ ros2 topic pub /goal_point geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 0.0}"

