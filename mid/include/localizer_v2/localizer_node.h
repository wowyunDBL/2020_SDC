/*data type*/
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>

/*read pcl*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
/*read file*/
#include <ros/package.h>
#include <dirent.h>
#include <string.h>
/*pcl convert to rosmsg*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
/*filter*/
#include <pcl/filters/voxel_grid.h>



struct pose{
	double x;
	double y;
	double z;
	double roll;
	double yaw;
	double pitch;
};

class Localizer{
  private:
	ros::Publisher pubMap;
	ros::Publisher pubLidar;
	tf::TransformBroadcaster br;
	pcl::PointCloud<pcl::PointXYZ> pclMap;
	struct pose posInitial,posGps;
	bool flagInit = false;
	bool flagGps = false;
  public:
	Localizer(ros::Publisher pubMap, ros::Publisher pubLidar, tf::TransformBroadcaster br, pcl::PointCloud<pcl::PointXYZ> fltMap);
	void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void gpsCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
	void save_data();
};
