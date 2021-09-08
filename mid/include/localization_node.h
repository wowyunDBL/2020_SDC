#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_datatypes.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <dirent.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include "read_pcl_file.h"
using namespace std;
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
	ros::Publisher map_pub;
	ros::Publisher pcl_pub;
	pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;
	tf::TransformBroadcaster br;
	double gps_x,gps_y,gps_z;
	int sec,nsec;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	bool gpsFlag,init;
	pose guess_pose,car2map_pose;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	ofstream outfile;
	geometry_msgs::TransformStamped lidar2car;
	
  public:
	Localizer(const ros::Publisher pub_map, const ros::Publisher pub_pcl, const pcl::PointCloud<pcl::PointXYZ>::Ptr &origin_map,const tf::TransformBroadcaster tf_br);
	void point_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void gnss_callback(const geometry_msgs::PointStamped &msg);
	void scanMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_ptr);
	void updatePose(struct pose &pose_now);
	void output_csv();
};
