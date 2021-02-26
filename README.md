# outdoor_gcs
This is the qt ground control station for drone with pixhawk mounted and ground station running mavros.

catkin build then 

```
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0

rosrun outdoor_gcs outdoor_gcs 
```

## This branch is created for multi-uav with px4_command running onboard
https://github.com/LonghaoQian/px4_command
