# outdoor_gcs
This is the qt ground control station for drone with pixhawk mounted and ground station running mavros.

catkin build then 

```
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0

rosrun outdoor_gcs outdoor_gcs 
```

## This branch is created for multi-uav for ubuntu 18.04 and qt5
Some unnecessary sections for testing purpose for single uav in master branch are eliminated.

And this qt gcs is extended for controlling multiple uavs.

Some codes are changed in CMakeLists.txt and package.xml for the package to be used in ubuntu 18.04 and qt5.
