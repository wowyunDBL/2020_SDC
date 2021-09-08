#include <ros/ros.h>
#include "localizer/localizer_node.h"

Localizer::Localizer(ros::Publisher pubMap, ros::Publisher pubLidar, tf::TransformBroadcaster br, pcl::PointCloud<pcl::PointXYZ> fltMap):pclMap(fltMap){
	this->pubMap = pubMap;
	this->pubLidar = pubLidar;
	this->br = br;
}

void Localizer::save_data(){
	ofstream outfile;
	outfile.open("result_public.txt",ios::app);
	outfile<<"x: "<<posInitial.x<<","<<"y: "<<posInitial.y<<endl;
	outfile.close();
}

void Localizer::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	ROS_INFO("enter lidar");
	if(flagGps){ //Enter after receive gps
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
	
		Eigen::AngleAxisf init_rotation_x(posInitial.roll, Eigen::Vector3f::UnitX() );
		Eigen::AngleAxisf init_rotation_y(posInitial.pitch, Eigen::Vector3f::UnitY() );
		Eigen::AngleAxisf init_rotation_z(posInitial.yaw, Eigen::Vector3f::UnitZ() );
		Eigen::Translation3f init_translation(posInitial.x,posInitial.y,posInitial.z);
		Eigen::Matrix4f init4icp = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrPclMap ( new pcl::PointCloud<pcl::PointXYZ>(pclMap) );
		pcl::PointCloud<pcl::PointXYZ> pclLidar;
		pcl::fromROSMsg(*msg, pclLidar);

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrPclLidar (new pcl::PointCloud<pcl::PointXYZ> (pclLidar));
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrFltLidar (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::VoxelGrid<pcl::PointXYZ> sor;
  		sor.setInputCloud (ptrPclLidar);
  		sor.setLeafSize (0.8f, 0.8f, 0.8f);
 		sor.filter (*ptrFltLidar);

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  		icp.setInputSource (ptrFltLidar);
  		icp.setInputTarget (ptrPclMap); 
		icp.setMaxCorrespondenceDistance(1);
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  		icp.align (*output, init4icp);
		Eigen::Matrix4d matTransform = Eigen::Matrix4d::Identity ();
		if ( icp.hasConverged() ){
			std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
			matTransform = icp.getFinalTransformation().cast<double>();
		}
		else{
			PCL_ERROR ("\nICP has not converged.\n");
			return ;
		}
		
		tf::Transform transform;
		tf::Quaternion q;
		posInitial.x = matTransform(0,3);
		posInitial.y = matTransform(1,3);
		posInitial.z = matTransform(2,3);
		transform.setOrigin( tf::Vector3(matTransform(0,3), matTransform(1,3), matTransform(2,3)) );

		tf::Matrix3x3 mat_tf(matTransform(0,0), matTransform(0,1), matTransform(0,2),
					matTransform(1,0), matTransform(1,1), matTransform(1,2),
					matTransform(2,0), matTransform(2,1), matTransform(2,2));
		mat_tf.getRPY(posInitial.roll, posInitial.pitch, posInitial.yaw);
		q.setRPY(posInitial.roll, posInitial.pitch, posInitial.yaw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "velodyne"));

		sensor_msgs::PointCloud2 msgLidar;
		pcl::toROSMsg(*ptrFltLidar, msgLidar);
		pubLidar.publish(msgLidar);

		sensor_msgs::PointCloud2 msgMap;
		pcl::toROSMsg(pclMap,msgMap);
		msgMap.header.frame_id = "map";
		pubMap.publish(msgMap);
		save_data();
	}
}

void Localizer::gpsCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	flagGps = true;
	posGps.x = msg->point.x;
	posGps.y = msg->point.y;
	posGps.z = msg->point.z;
}

void read_pcl_map(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered){
	std::string path = ros::package::getPath("localizer");
	path += "/map/";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	struct dirent *dp;
	DIR* dirp = opendir(path.c_str());
	while( (dp = readdir(dirp)) != NULL){
		if(strstr(dp->d_name, ".pcd") != NULL){
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (path+dp->d_name, *cloud) == -1)  //* load the file
    				PCL_ERROR ("Couldn't read .pcd \n");
			std::cout << "Loaded " << dp->d_name << std::endl;
  		}
	}
  	
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (0.8f, 0.8f, 0.8f);
 	sor.filter (*cloud_filtered);
}
int main(int argc, char **argv){
	ros::init(argc, argv, "localizer_node");
	ros::NodeHandle nh;
	ros::Publisher pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map",10);
	ros::Publisher pubLidar = nh.advertise<sensor_msgs::PointCloud2>("/lidar",10);
	tf::TransformBroadcaster br;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
	read_pcl_map(cloud_filtered);

	Localizer *locObj = new Localizer(pubMap, pubLidar, br, *cloud_filtered);
	ros::Subscriber subLidar = nh.subscribe("/lidar_points", 10, &Localizer::lidarCallback, locObj);
	ros::Subscriber subGps = nh.subscribe("/fix", 10, &Localizer::gpsCallback, locObj);

	ros::spin();
	return 0;
}
