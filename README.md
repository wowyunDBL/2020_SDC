# 2020_Self Driving Car
This is the homework repository for Self Driving Car.

## Code Implementation
/* Build packages */
```sh
$ cd $(CATKIN_WS)
$ catkin_make
$ source devel/setup.bash
$ rosrun localizer localization_node
```

/* Play rosbag */
```sh
$ rosparam set use_sim_time true
$ rosbag play -r 0.1 {challenge}.bag --clock
```

/* Rviz topic */
```sh
frame: map
topic: sensor_map
topic: sensor_scan
```

## HW1-Introduction

## HW2-Probability
### Reference
* ProbRobotics
    * ch1 Introduction
        * key word: Markow localization、Bayes filter
    * ch2 Recursive State Estimation
        * key word: dynamic Bayes network
	* ch7 Recursive State Estimation
        * key word: Markov Localization

## HW3-Visualization Fusion Trajectory
### key point 
* [hw3 requirement](https://github.com/wowyunDBL/2020_SDC/blob/master/hw3_0511009/Assignment3_IMU.pdf)
* [An introduction to inertial navigation](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf)
* Use the angular velocity and linear acceleration to draw a path.

## HW4-Tracking
### key point 
* [hw4 requirement](https://github.com/wowyunDBL/2020_SDC/blob/master/hw4/hw4_0511009/Assignment%204.pdf)
* Use EKF to do sensor fusion 
* Visual odometry:以影像作為input，經過影響處理，還原相機在拍時的位移、旋轉
* Transformation imu frame to zed odom frame
* Red:visual odem、Green:EKF、Blue:Imu
![image](https://github.com/wowyunDBL/2020_SDC/blob/master/images/hw4.png)

## Competition
### Midterm-Localization
#### key point 
* using C++ language
* ICP (iterative_closet_point)
input point cloud map(.pcd) as target cloud of ICP
* TF (transform)
* load and save file
* [self-practice with code explanation tutorial]
#### Reference
* 3D Point Cloud Processing and Learning for Autonomous Driving
* [See code detail](https://github.com/wowyunDBL/2020_SDC/blob/master/mid/README.md)
![image](https://github.com/wowyunDBL/2020_SDC/blob/master/images/mid.png)

### Final
#### key point 
* using python language
* tracking and forecasting using argo-dataset with on-line baseline
* draw bounding box and distinguish different class
#### Reference 
* Probabilistic 3D Multi-Object Tracking for Autonomous Driving
[video link](https://www.youtube.com/watch?v=BZEGhJdUKMM)
![image](https://github.com/wowyunDBL/2020_SDC/blob/master/images/final.png)
