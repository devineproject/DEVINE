Image Segmentation
============================

## Installation
1. Run the install script `source install_package.bash`
2. Install [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation) for python 2.
3. Build the module using catkin\_make:
```bash
roscd
cd ..
catkin_make
```

## Usage
```bash
rosrun devine_image_processing segmentation.py
rosrun devine_image_processing features_extraction.py
```
