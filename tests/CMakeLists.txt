cmake_minimum_required(VERSION 2.8.3)
project(devine_tests)
find_package(catkin REQUIRED)

catkin_python_setup()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
  add_rostest(test/default.test)
endif()
