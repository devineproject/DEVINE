# IRL-1 Control
ROS package developed in Python to send IRL-1:
- JointTrajectory to rigth and left arms
- JointTrajectory to head
- Command to left and right gripper

## Compiling
```bash
cd ~/catkin_ws
catkin_make
```

## Usage Example
```bash
rosrun irl_control example.py -r jn0 -c left_arm_controller -p [-0.5,0,-1,-1] -t 5
```

## Details
File `irl_constant.py` contains
- Controllers names
- Joints names
- Joints limits

See `package.xml` for dependancies
