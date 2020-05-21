/* hw4: Combine imu data and visual odometry to do sensor fusion by using EKF

   1. Visualiza the path of IMU sensor and ZED stereo camera and result from robot_pose_ekf 
   2. Transform imu frame to zed odom frame
   
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen_conversions/eigen_msg.h>
using namespace std;

ros::Publisher marker_imu_pub,marker_vo_pub,marker_ekf_pub, imu_data;
visualization_msgs::Marker line_imu,line_vo,line_ekf;
geometry_msgs::Point p_imu,p_vo,p_ekf;

Eigen::Vector3d v_past,v_now;
Eigen::Vector3d s_past,s_now; 
Eigen::Vector3d g_g,a_g,a_b;
Eigen::Vector3d w;
Eigen::Matrix3d C,C_past,B;
double sigma;
double time_past,time_now,dt;
bool flag = true;

void initial_marker();
void pub_marker_imu();
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
void voCallback(const nav_msgs::Odometry::ConstPtr& msg);
void ekfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);	

int main(int argc, char** argv)
{
	ros::init(argc,argv,"hw4_node");
	ros::NodeHandle n;
	ros::Subscriber imu_sub = n.subscribe("/imu/data",10,imuCallback);
	marker_imu_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker_imu",10);
	imu_data = n.advertise<sensor_msgs::Imu>("/imu_data",10);
	
	ros::Subscriber vo_sub = n.subscribe("/zed/odom",1000,voCallback);
	marker_vo_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker_vo",10);
	
	ros::Subscriber ekf_sub = n.subscribe("/robot_pose_ekf/odom_combined",1000,ekfCallback);
	marker_ekf_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker_ekf",10);
	
	initial_marker();
	
	ros::spin();       //wait for the subscriber
	return 0;
}
void initial_marker()
{
	line_imu.header.frame_id = "map";
	line_imu.header.stamp = ros::Time::now();
	line_imu.action = visualization_msgs::Marker::ADD;
	line_imu.pose.orientation.w = 1.0;
	line_imu.id = 0;
	line_imu.type = visualization_msgs::Marker::LINE_STRIP;
	line_imu.scale.x = 0.1;
	line_imu.color.b = 1.0;
	line_imu.color.a = 1.0;

	line_vo.header.frame_id = "map";
	line_vo.header.stamp = ros::Time::now();
	line_vo.action = visualization_msgs::Marker::ADD;
	line_vo.pose.orientation.w = 1.0;
	line_vo.id = 1;
	line_vo.type = visualization_msgs::Marker::LINE_STRIP;
	line_vo.scale.x = 0.1;
	line_vo.color.r = 1.0;
	line_vo.color.a = 1.0;

	line_ekf.header.frame_id = "map";
	line_ekf.header.stamp = ros::Time::now();
	line_ekf.action = visualization_msgs::Marker::ADD;
	line_ekf.pose.orientation.w = 1.0;
	line_ekf.id = 2;
	line_ekf.type = visualization_msgs::Marker::LINE_STRIP;
	line_ekf.scale.x = 0.1;
	line_ekf.color.g = 1.0;
	line_ekf.color.a = 1.0;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{	
	sensor_msgs::Imu newImu;
	Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg->orientation,q);      
	/*q.w() = msg->orientation.w;
	q.x() = msg->orientation.x;
	q.y() = msg->orientation.y;
	q.z() = msg->orientation.z;
	*/                   
	Eigen::Matrix3d q2r = q.normalized().toRotationMatrix();           //change quaternion to rotation

	Eigen::Vector3d w;
	tf::vectorMsgToEigen(msg->angular_velocity,w);
	//w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	
	Eigen::Vector3d a;
	tf::vectorMsgToEigen(msg->linear_acceleration,a);
	//a << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

	Eigen::Matrix3d R_imu2cam;
	R_imu2cam << 0.0225226, 0.999745, 0.0017194,
		     0.0648765, -0.00317777, 0.997888,
		     0.997639, -0.0223635, -0.0649315;
	Eigen::Matrix3d R_cam2zed;
	R_cam2zed << 0, 0, 1,
		     -1, 0, 0,
		      0, -1, 0;
	q2r = R_imu2cam * q2r;
	q2r = R_cam2zed * q2r;
	w = R_imu2cam * w;
	w = R_cam2zed * w;
	a = R_imu2cam * a;
	a = R_cam2zed * a;
	
	Eigen::Quaterniond q1 (q2r);                                    //change rotation to quaternion
	tf::quaternionEigenToMsg(q1,newImu.orientation);
	tf::vectorEigenToMsg(w,newImu.angular_velocity);
	tf::vectorEigenToMsg(a,newImu.linear_acceleration);
	/*newImu.orientation.w = q1.w();
	newImu.orientation.x = q1.x();
	newImu.orientation.y = q1.y();
	newImu.orientation.z = q1.z();
	newImu.angular_velocity.x = w(0);
	newImu.angular_velocity.y = w(1);
	newImu.angular_velocity.z = w(2);
	newImu.linear_acceleration.x = a(0);
	newImu.linear_acceleration.y = a(1);
	newImu.linear_acceleration.z = a(2);
	*/
	newImu.header = msg->header;	                                //time stamp will wrong

	imu_data.publish(newImu);	
	
	if(flag)
	{
		flag = false;
		time_now = msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec;
		s_now << 0,0,0; 
		v_now << 0,0,0;
		C = Eigen::Matrix3d::Identity();
		g_g << newImu.linear_acceleration.x, newImu.linear_acceleration.y, newImu.linear_acceleration.z; 
	}	
	else 
	{
		time_now = msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec;
		dt = time_now - time_past;
		w << newImu.angular_velocity.x, newImu.angular_velocity.y, newImu.angular_velocity.z;
		sigma = (w*dt).norm();
		B << 0,-w(2)*dt,w(1)*dt,
		     w(2)*dt,0,-w(0)*dt,
		     -w(1)*dt,w(0)*dt,0;
		C = C_past*(Eigen::Matrix3d::Identity() + sin(sigma)/sigma*B + (1-cos(sigma))/pow(sigma,2)*B*B);
		a_b << newImu.linear_acceleration.x, newImu.linear_acceleration.y, newImu.linear_acceleration.z;
		a_g = C*a_b;
		v_now = v_past + dt*(a_g - g_g);
		s_now = s_past + dt*v_now;
	
	}
	pub_marker_imu();

	time_past = time_now;
	v_past = v_now;
	s_past = s_now;
	C_past = C;
}

void voCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	p_vo.x = msg->pose.pose.position.x;
	p_vo.y = msg->pose.pose.position.y;
	p_vo.z = msg->pose.pose.position.z;
	line_vo.points.push_back(p_vo);
	marker_vo_pub.publish(line_vo);
}	

void ekfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	p_ekf.x = msg->pose.pose.position.x;
	p_ekf.y = msg->pose.pose.position.y;
	p_ekf.z = msg->pose.pose.position.z;
	line_ekf.points.push_back(p_ekf);
	marker_ekf_pub.publish(line_ekf);
}	



void pub_marker_imu()
{
	p_imu.x = s_now(0);
	p_imu.y = s_now(1);
	p_imu.z = s_now(2);
	line_imu.points.push_back(p_imu);
	marker_imu_pub.publish(line_imu);
}


