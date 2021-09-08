# Learning C++ by Localization task
## 1. create ROS architecture
### include header file
```cpp=
#include <ros/ros.h>
#include <std_msgs/String.h>
```
### rosinit
```cpp=
int main(int argc, char **argv){
	ros::init(argc, argv, "localizer_node");
	ros::NodeHandle nh;
	ros::Publisher pubMap = nh.advertise<std_msgs::String>("/wow",10);
	ros::Subscriber subMap = nh.subscribe("/wow", 10, &msgCallback);
	
	std_msgs::String msg;
	msg.data = "wow";
	pubMap.publish(msg);

	ros::spin();
	return 0;
}
```
### callback function
```cpp=
void msgCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```
### naming and layout rules
* Publisher: pubMap
* Subscriber: subMap
* callback: lidarCallback

## 2. Read map file (.pcd)
### c++ read directory
```cpp=
struct dirent *dp;
DIR* dirp = opendir(path.c_str());
while( (dp = readdir(dirp)) != NULL){}
```
### get ROS package path
```cpp=
string path = ros::package::getPath("localizer");
path += "/map/";
```
### [PCL read](http://pointclouds.org/documentation/tutorials/reading_pcd.html#reading-pcd)
```cpp=
if(strstr(dp->d_name, ".pcd") != NULL){
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path+dp->d_name, *cloud) == -1){ //* load the file
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
    }
}
```
### pubish MAP in RVIZ
```cpp=
sensor_msgs::PointCloud2 msgMap;
pcl::toROSMsg(*cloud,msgMap);
msgMap.header.frame_id = "/map";
pubMap.publish(msgMap);
```
![](https://i.imgur.com/hW1UbIj.png)
The map size is too large, and the hard ware resource is limited, so it capture the time with pink color.

### After applying voxel_filter
```cpp=
sor.setLeafSize (1.0f, 1.0f, 1.0f);
```
![](https://i.imgur.com/2FSRa5B.png)

## 3. Read lidar msg from rosbag
### Callback function and tf
```cpp=
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.46,0,3.46));
	tf::Quaternion q(-0.0051505,0.018102,-0.019207,-0.99964);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "velodyne"));

	pubLidar.publish(*msg);
}
```

* Getting transform and rotation info 

    * $ rostopic echo /tf

![](https://i.imgur.com/bFnB5sH.png)

### After successfully publishing tf 
![](https://i.imgur.com/8qKKPnF.png)


## 4. Packaging like a class
### localizer object
* receive mapPublisher and lidarPublisher and br
* preprocess map point cloud
* be careful the function type of subscribe()
```cpp=
Localizer *locObj = new Localizer(pubMap, pubLidar, br, *cloud_filtered);
ros::Subscriber subLidar = nh.subscribe("/lidar_points",10,&Localizer::lidarCallback,locObj);
```
### constructor
```cpp=
Localizer::Localizer(ros::Publisher pubMap, ros::Publisher pubLidar, tf::TransformBroadcaster br, pcl::PointCloud<pcl::PointXYZ> fltMap)
:pclMap(fltMap){
    this->pubMap = pubMap;
    this->pubLidar = pubLidar;
    this->br = br;
}
```
### move out function and ptr parameter
```cpp=
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
read_pcl_map(cloud_filtered);
```
## 5. Subscribe gnss msg
* in order to get initial pose for ICP
### set initial pose using struct
```cpp=
struct pose{
    double x;
    double y;
    double z;
    double roll;
    double yaw;
    double pitch;
};
//change br data
transform.setOrigin(tf::Vector3(posInitial.x,posInitial.y,posInitial.z));
```
now we can see lidar in the favor place!
![](https://i.imgur.com/XJhdyOz.png)

## 6. implement ICP
### iterative closet point
* pipe line: use GPS as first guess
```cpp=
if(!flagInit){
    ROS_INFO("receive gps");
    posInitial.x = posGps.x;
    posInitial.y = posGps.y;
    posInitial.z = posGps.z+0.89;
    posInitial.roll = 0;
    posInitial.yaw = -2.199;
    posInitial.pitch = 0;
    flagInit = true;
}
```
### data type transform from eigen to tf
```cpp=
Eigen::Matrix4d matTF;
tf::Transform transform;
tf::Quaternion q;
tf::Matrix3x3 mat_tf(matTF(0,0), matTF(0,1), matTF(0,2),
					matTF(1,0), matTF(1,1), matTF(1,2),
					matTF(2,0), matTF(2,1), matTF(2,2));
mat_tf.getRPY(posResult.roll, posResult.pitch, posResult.yaw);
q.setRPY(posResult.roll, posResult.pitch, posResult.yaw);
transform.setOrigin( tf::Vector3(matTF(0,3), matTF(1,3), matTF(2,3) );
transform.setRotation(q);
```
### use axis rotation to set transformation
```cpp=
Eigen::AngleAxisf init_rotation_x(posInitial.roll, Eigen::Vector3f::UnitX() );
Eigen::AngleAxisf init_rotation_y(posInitial.pitch, Eigen::Vector3f::UnitY() );
Eigen::AngleAxisf init_rotation_z(posInitial.yaw, Eigen::Vector3f::UnitZ() );
Eigen::Translation3f init_translation(posInitial.x,posInitial.y,posInitial.z);
Eigen::Matrix4f init4icp = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
```
### the frist step is important
* check every angle to set the first guess

## 7. save data with transform matrix
```cpp=
void Localizer::save_data(){
    ofstream outfile;
    outfile.open("result_public.txt",ios::app);
    outfile<<"x: "<<posInitial.x<<","<<"y: "<<posInitial.y<<endl;
    outfile.close();
}
```
