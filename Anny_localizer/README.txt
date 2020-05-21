/* Build packages */
$ cd $(CATKIN_WS)
$ catkin_make
$ source devel/setup.bash
$ rosrun localizer localization_node

/* Play rosbag */
$ rosparam set use_sim_time true
$ rosbag play -r 0.1 {challenge}.bag --clock

/* Rviz topic */
frame: map
topic: sensor_map
topic: sensor_scan
