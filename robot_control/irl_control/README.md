# IRL-1 Control
Python ROS package to control IRL-1 mouvements

# Overview
- Point to position (x, y, z) with
  - right arm
  - left arm
  - head
- Open and close
  - right gripper
  - left gripper
- TODO Facial expression
- SIMULATION ONLY: Do complex movements with arms and head:
    - happy (confidence >= threshold, success 1)
    - satisfied (confidence < threshold, success 1)
    - disappointed (confidence >= threshold, success 0)
    - sad (confidence < threshold, success 0)

# Usage Example

## 0. Compiling
```bash
cd ~/catkin_ws
catkin_make
```

## 1. Before using examples, launch IRL-1
Launch jn0 with RViz UI and irl_control nodes
```bash
roslaunch irl_control irl_control.launch
```

## 2. Load RVIZ configuration
```File -> Open Config -> irl_point.rviz```

## 3. Execute examples scripts
### Point to position [x, y, z]
```bash
rosrun irl_control example_point.py -p 0.6,0.3,0.5
# Position is referenced from base_link
```

### Do complex move (SIMULATION ONLY!!!)

```bash
rosrun irl_control example_emotion.py -c 0 -s 0
```

# Details
Arms controller use `JointTrajectory`

Head controller use `JointTrajectory`

Grippers use `Command`

`marker.py` use:
- `MarkerArray` to show position to point in RVIZ
- `TF` to broadcast position to point and to calculate the orientations error (should be +/- 5 degrees).

`example.py` use:
- `TF` to listen position between `/[right|left]_shoulder_fixed_link` and position to point `/obj`
- `ik.py` to get joint position knowing position to point (see documentation from Arnaud Aumont, IntRoLab, version 22/01/2012)


## Constant
File `irl_constant.py` contains
- Controllers names
- Joints names
- Joints limits

## Dependencies
See `package.xml` for dependencies
