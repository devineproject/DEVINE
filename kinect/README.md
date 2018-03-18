Kinect setup 
==================

1. Install OpenNI
```bash
sudo apt-get install ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description
sudo apt-get install ros-kinetic-compressed-image-transport ros-kinetic-compressed-image-transport-plugins #Image compression plugin
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

4. Read documentation for more !
http://wiki.ros.org/openni_launch/
