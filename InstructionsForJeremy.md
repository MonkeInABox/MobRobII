# Hello these are my instructions, not a ReadMe

## TF2 
- Is used to add the base link adn base laser frames to a transform tree
- Each one being a coordinate frame

```
ros2 run tf2_ros static_transform_publisher --x 0.2 --y 0 --z 0.104 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser_frame

in a new cmd

ros2 run tf2_ros tf2_echo base_link laser_frame
```

- This is actually already in my URDF so don't need to worry about it

- NAV2 adheres to https://www.ros.org/reps/rep-0105.html and https://www.ros.org/reps/rep-0103.html

- Odom coord frame is relatve to robots starting position
- Map frame is a world fixed frame used for globally ocnsistent representations

- Z up X forward

### NAV2 REQUIRES TRANSFORMS
- map -> odom
- odom -> baselink
- baselink -> base_laser etc

# Run
```
colcon build
. install/setup.bash


then:

ros2 launch pioneer display.launch.py
```

# nav_msgs/Odometry Message
```
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```


# Random errors: do this 
```
unset GTK_PATH
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/snap/[^:]*:||g')
ros2 launch pioneer display.launch.py
```

# look at tree
```
ros2 run tf2_tools view_frames

firefox frames_2026-03-26_09.18.41.pdf 
```