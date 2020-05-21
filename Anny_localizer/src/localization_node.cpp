/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  sensor_msgs::PointCloud2
*/
#include "localization_node.h"


Localizer::Localizer(const ros::Publisher pub_map, const ros::Publisher pub_pcl, const pcl::PointCloud<pcl::PointXYZ>::Ptr &origin_map,tf::TransformBroadcaster tf_br)
{
	map_pub = pub_map;
	pcl_pub = pub_pcl;
	map_ptr = origin_map;
	gpsFlag = false;
	init = false;
	br = tf_br;
	string fileName = "result_public.csv";
	outfile.open(fileName.c_str(),ofstream::app);
	
	lidar2car.transform.translation.x = 0.46;
	lidar2car.transform.translation.y = 0.0;
	lidar2car.transform.translation.z = 3.46;
	lidar2car.transform.rotation.x = -0.0051505;
	lidar2car.transform.rotation.y = 0.018102;
	lidar2car.transform.rotation.z = -0.019207;
	lidar2car.transform.rotation.w = 0.99964;

	//Downsampling
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZ>() );
	sor.setInputCloud(origin_map);
	sor.setLeafSize(1,1,1);
	sor.filter(*filtered_map);
		//map_ptr = filtered_map;			
	cout<<"filtered map!\n";
}
void Localizer::point_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	
	sec = msg->header.stamp.sec;
	nsec = msg->header.stamp.nsec;

	if(!gpsFlag)
		return ;
	pcl::PointCloud<pcl::PointXYZ> scan;
	pcl::fromROSMsg(*msg,scan);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan) );
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZ>() );
	sor.setInputCloud(scan_ptr);
	sor.setLeafSize(1,1,1);
	sor.filter(*filtered_scan);
	
	scanMatching(filtered_scan);

	sensor_msgs::PointCloud2 sensor_scan;
	//pcl::toROSMsg(*filtered_scan, sensor_scan);
	sensor_scan = *msg;
	sensor_scan.header.frame_id = "velodyne1";
	pcl_pub.publish(sensor_scan);
	cout<<"publish lidar success!\n";

	sensor_msgs::PointCloud2 sensor_map;
	pcl::toROSMsg(*map_ptr,sensor_map);
	sensor_map.header.frame_id = "map";
	map_pub.publish(sensor_map);
}
void Localizer::scanMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_ptr)
{	
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>() );
	if(!init)
	{
		init = true;
		/*double min_score=1000,min_yaw;
		pose min_pose;
		for(double yaw=0; yaw<6.3; yaw += 0.4)
		{
			pose initial_guess = {gps_x,gps_y,gps_z,0,yaw,0};
			Eigen::AngleAxisf init_rotation_x(initial_guess.roll, Eigen::Vector3f::UnitX() );
			Eigen::AngleAxisf init_rotation_y(initial_guess.pitch, Eigen::Vector3f::UnitY() );
			Eigen::AngleAxisf init_rotation_z(initial_guess.yaw, Eigen::Vector3f::UnitZ() );
			Eigen::Translation3f init_translation(initial_guess.x, initial_guess.y, initial_guess.z);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
			
			//set ICP
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1;
			icp1.setInputSource(scan_ptr);
			icp1.setInputTarget(map_ptr);
			icp1.setMaxCorrespondenceDistance(2.5);
			icp1.setMaximumIterations(1000);
			icp1.setEuclideanFitnessEpsilon(0.0001);
			icp1.setTransformationEpsilon(1e-9);
			cout<<"\n-----------start aligning yaw: "<< yaw << endl;
			icp1.align(*output,init_guess);
			double score = icp1.getFitnessScore(0.5);
			cout<<"min score: "<< min_score <<" score: " << score;
			if(min_score > score)
			{
				min_score = score;
				min_yaw = yaw;
			}
			
		}*/
		
		guess_pose.x = gps_x;
		guess_pose.y = gps_y;
		guess_pose.z = gps_z+0.89;
		guess_pose.roll = 0;
		guess_pose.yaw = -2.199;
		guess_pose.pitch = 0;
		
	}
	
		
	Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX() );
	Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY() );
	Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ() );
	Eigen::Translation3f init_translation(guess_pose.x,guess_pose.y,guess_pose.z);
	Eigen::Matrix4f guessICP = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
	
	icp.setInputSource(scan_ptr);
	icp.setInputTarget(map_ptr);
	icp.setMaxCorrespondenceDistance(1);
	icp.align(*output,guessICP);
	Eigen::Matrix4f T(Eigen::Matrix4f::Identity() );
	T = icp.getFinalTransformation();
	
	pose poseNow;
	poseNow.x = T(0,3);       
	poseNow.y = T(1,3);
	poseNow.z = T(2,3);
	tf::Matrix3x3 rotationNow;
	rotationNow.setValue(T(0,0),T(0,1),T(0,2),
				T(1,0),T(1,1),T(1,2),
				T(2,0),T(2,1),T(2,2) );
	rotationNow.getRPY(poseNow.roll, poseNow.pitch, poseNow.yaw);

	tf::Transform transform;
        transform.setOrigin( tf::Vector3(poseNow.x,poseNow.y,poseNow.z) );
        tf::Quaternion q;
	q.setRPY(poseNow.roll, poseNow.pitch, poseNow.yaw);
	transform.setRotation(q);
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne1") );

	updatePose(poseNow);	

	//transform the result back to baselink
	Eigen::Affine3d M_lidar2car,M_lidar2map;
	M_lidar2map.matrix() = T.cast<double>();
	M_lidar2car = (tf2::transformToEigen(lidar2car) ); 
	Eigen::Affine3d M_car2map = M_lidar2map * M_lidar2car.inverse();
	tf::Transform transform_tf;
	tf::transformEigenToTF(M_car2map,transform_tf);
	tf::Quaternion qq = transform_tf.getRotation();
	tf::Matrix3x3 mm (qq);
	tfScalar yaw,pitch,roll;
	mm.getEulerYPR(car2map_pose.yaw, car2map_pose.pitch, car2map_pose.roll);
	car2map_pose.x = M_car2map.translation().x();
	car2map_pose.y = M_car2map.translation().y();
	car2map_pose.z = M_car2map.translation().z();
	output_csv();
	//outfile << poseNow.x <<','<< poseNow.y << ',' << poseNow.z << ','<< guess_pose.yaw << endl;
}
void Localizer::output_csv()
{
	outfile<< to_string(sec) <<'.' << setw(9) << setfill('0') << nsec <<',' << car2map_pose.x <<','<< car2map_pose.y << ',' << car2map_pose.z << ','<< car2map_pose.yaw << ','<< car2map_pose.pitch << ','<< car2map_pose.roll << endl;
}
void Localizer::updatePose(struct pose &pose_now)
{
	guess_pose.x = pose_now.x;
	guess_pose.y = pose_now.y;
	guess_pose.z = pose_now.z;
	guess_pose.roll = pose_now.roll;
	guess_pose.yaw = pose_now.yaw;
	guess_pose.pitch = pose_now.pitch;
}
void Localizer::gnss_callback(const geometry_msgs::PointStamped &msg)
{	
	gpsFlag = true;
	gps_x = msg.point.x;
	gps_y = msg.point.y;
	gps_z = msg.point.z;
	cout<<"subscribe GPS success!\n";
}
int main(int argc, char **argv)
{	
	ros::init(argc,argv,"localization_node");
	ros::NodeHandle nh;
	ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor_map",1000);
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan",1000);
	tf::TransformBroadcaster br;	

	string path = ros::package::getPath("localizer");
	path += "/map/";
	pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>() );
	read_pcl_file(path, map);
		
	Localizer *lObj = new Localizer(map_pub,pcl_pub,map,br);
	ros::Subscriber lidar_sub = nh.subscribe("/lidar_points",1000, &Localizer::point_callback,lObj);	
	ros::Subscriber gps_sub = nh.subscribe("/fix",1000,&Localizer::gnss_callback,lObj);
	ros::spin();
	return 0;
}
