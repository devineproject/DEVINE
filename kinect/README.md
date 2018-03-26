Kinect setup 
==================

1. Install OpenNI
```bash
sudo apt-get install ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description
sudo apt-get install ros-kinetic-compressed-image-transport #Image compression plugin
```

2. Start OpenNI server
```bash
roslaunch openni_launch openni.launch 
```

3. View Data
You can use the dashboard or the image_view package:
```bash
rosrun image_view image_view image:=/camera/rgb/image_color #color
rosrun image_view image_view image:=/camera/rgb/image_mono #mono
rosrun image_view disparity_view image:=/camera/depth_registered/disparity #disparity
```

4. Read documentation for more info on openni !
http://wiki.ros.org/openni_launch/

# Devine Kinect Package
Specialized ROS package for the integration

## Installation
1. Run the install script `./install_package.bash` 
2. Build the module using catkin_make:
```bash
roscd
cd ..
catkin_make
```

## Usage
```bash
rosrun devine_kinect pos_lib.py #run devine_kinect node

#publish a location:
rostopic pub /object_found std_msgs/Int32MultiArray "layout:
 dim: []
 data_offset: 0
data: [320,240]"

#listen to calculated positions
rostopic echo /object_location
``` 
