# IRL-1 Control
Python ROS package to point position (x, y, z) by using:
- `JointTrajectory` for right and left arms controllers
- `JointTrajectory` for head controller
- `Command` for left and right grippers

## Compiling
```bash
cd ~/catkin_ws
catkin_make
```

## Usage Example

### Before using examples, launch IRL-1
```bash
roslaunch jn0_gazebo jn0_empty_world.launch
```
Launch Gazebo
```bash
roslaunch jn0_gazebo jn0_empty_world.launch gui:=true
```

### Load RVIZ configuration
```File -> Open Config -> irl_point.rviz```

### Point object with position [x, y, z]
 Position is referenced from base_link

```bash
rosrun irl_control marker.py -p 0.6,0.3,0.5

rosrun irl_control example.py -r jn0 -c right_arm_controller -p 0.6,0.3,0.5 -t 5
```

### Move controller with joints position
```bash
rosrun irl_control example.py -r jn0 -c left_arm_controller -j -0.5,0,-1,-1 -t 5
```

## Details
`marker.py` use:
- `MarkerArray` to show position to point in RVIZ
- `TF` to broadcast position to point and to calculate the orientations error (should be +/- 5 degrees).

`example.py` use:
- `TF` to listen position between `/[right|left]_shoulder_fixed_link` and position to point `/obj`
- `ik.py` to get joint position knowing position to point (see documentation from Arnaud Aumont, IntRoLab, version 22/01/2012)


### Constant
File `irl_constant.py` contains
- Controllers names
- Joints names
- Joints limits

### Dependencies
See `package.xml` for dependencies
