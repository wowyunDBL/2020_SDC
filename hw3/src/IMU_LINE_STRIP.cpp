/* hw3: Visualize the path of IMU in rviz
   1. Subscribe topic /imu/data 
   2. Use the angular velocity and linear acceleration to draw a path
   3. Use LINE_STRIP marker to draw the path
   4. Initial: s_g(0) = (0,0,0); C(0) = I; 
      Gravity vector in global frame is const equal to the first measurement
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <Eigen/Dense>

using namespace std;

ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;
geometry_msgs::Point p;
double time_past,time_now,dt;
Eigen::Vector3d v_past,v_now;
Eigen::Vector3d s_past,s_now; 
Eigen::Vector3d g_g,a_g,a_b;
Eigen::Vector3d w;
Eigen::Matrix3d C,C_past,B;
double sigma;
bool flag = true;

void initial_marker();
void pub_marker_rviz();
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	
int main(int argc, char** argv)
{
	ros::init(argc,argv,"hw3_node");
	ros::NodeHandle n;
	ros::Subscriber imu_sub = n.subscribe("/imu/data",10,imu_callback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker",10);
	initial_marker();
	
	ros::spin();       //wait for the subscriber
	return 0;
}
void initial_marker()
{
	line_strip.header.frame_id = "map";
	line_strip.header.stamp = ros::Time::now();
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 0;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.1;
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;
}
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(flag)
	{
		flag = false;
		time_now = msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec;
		s_now << 0,0,0; 
		v_now << 0,0,0;
		C = Eigen::Matrix3d::Identity();
		g_g << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; 
	}	
	else 
	{
		time_now = msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec;
		dt = time_now - time_past;
		w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
		sigma = (w*dt).norm();
		B << 0,-w(2)*dt,w(1)*dt,
		     w(2)*dt,0,-w(0)*dt,
		     -w(1)*dt,w(0)*dt,0;
		C = C_past*(Eigen::Matrix3d::Identity() + sin(sigma)/sigma*B + (1-cos(sigma))/pow(sigma,2)*B*B);
		a_b << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
		a_g = C*a_b;
		v_now = v_past + dt*(a_g - g_g);
		s_now = s_past + dt*v_now;
	
	}
	pub_marker_rviz();

	time_past = time_now;
	v_past = v_now;
	s_past = s_now;
	C_past = C;
}
void pub_marker_rviz()
{
	p.x = s_now(0);
	p.y = s_now(1);
	p.z = s_now(2);
	line_strip.points.push_back(p);
	marker_pub.publish(line_strip);
}


