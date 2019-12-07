#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>

#include <math.h>


#define A (36.5/100)     //车宽一半 m
#define B (44.64/100)    //车长一半 m




double e1,e2,e3,e4;

ros::Time current_time,last_time;
bool first = 1;

ros::Publisher pub;
nav_msgs::Odometry od;

double odom_x = 0;
double odom_y = 0;
double odom_z = 0;
double odom_last6 = 0.0f;
double odom_last7 = 0.0f;
double delta_x = 0.0f;
double delta_y = 0.0f;
double delta_z = 0.0f;
double v1 = 0.0f;
double v2 = 0.0f;
double theta1 = 0.0f;
double theta2 = 0.0f;
double v1s = 0.0f;
double v2s = 0.0f;
double v1c = 0.0f;
double v2c = 0.0f;
double dt = 0.0f;



geometry_msgs::TransformStamped od_tf;
geometry_msgs::Quaternion odom_quat;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void encoder_Callback(const geometry_msgs::TwistStamped& msg)
{
	e1 = -msg.twist.linear.x ;
	e2 = msg.twist.linear.y ;
	e3 = msg.twist.angular.x;
	e4 = msg.twist.angular.y;
	current_time = msg.header.stamp;
	if (first == 1){
		first = 0;
		last_time = current_time;
	}
	dt = (current_time - last_time).toSec();
	if (dt != 0.0f){
		v1 = (e1 - odom_last7)/dt;
		v2 = (e2 - odom_last6)/dt;
		theta1 = e3;
		theta2 = e4;
	}
	odom_last7 = e1;
	odom_last6 = e2;

	v2c = v2*cos(theta2);
	v1c = v1*cos(theta1);
	v2s = v2*sin(theta2);
	v1s = v1*sin(theta1);
	//printf("%f,%f,%f,%f\n",v1s,v2s,v1c,v2c);
	
	delta_z = 0.5 * ( v1c -v2c )/A;
	delta_x = 0.5 * (v2s + v1s) - delta_z * B;
	delta_y = 0.5 * (v2c + v1c);
	//printf("%f,%f,%f,%f\n",dt,delta_z,delta_y,delta_x);
	

	odom_x += delta_x * dt * cos(odom_z) - delta_y * dt * sin(odom_z);
	odom_y += delta_y * dt * cos(odom_z) + delta_x * dt * sin(odom_z);
	odom_z += delta_z * dt;
	//printf("%f,%f,%f,%f\n",dt,odom_x,odom_y,odom_z);


	odom_quat = tf::createQuaternionMsgFromYaw(odom_z);
/*
	od_tf.header.stamp = current_time;
	od_tf.header.frame_id = "odom";
	od_tf.child_frame_id= "car_link";

	od_tf.transform.translation.x = odom_y;
	od_tf.transform.translation.y = -odom_x;
	od_tf.transform.translation.z = 0.0f;
	od_tf.transform.rotation = odom_quat;

	od_brod.sendTransform(od_tf);
*/
	od.header.stamp = current_time;
	od.header.frame_id = "odom";

	od.pose.pose.position.x = odom_y;
	od.pose.pose.position.y = -odom_x;
	od.pose.pose.position.z = 0.0f;
	od.pose.pose.orientation = odom_quat;

	od.child_frame_id = "car_link";
	od.twist.twist.linear.x = delta_y;
	od.twist.twist.linear.y = -delta_x;
	od.twist.twist.angular.z = delta_z;

	od.pose.covariance[0] = 1e-3;
	od.pose.covariance[7] = 1e-3;
	od.pose.covariance[14] = 1e6;
	od.pose.covariance[21] = 1e6;
	od.pose.covariance[28] = 1e6;
	od.pose.covariance[35] = 1e3;

	od.twist.covariance[0] =1e-3;
	od.twist.covariance[7] = 1e-3;
	od.twist.covariance[14] = 1e6;
	od.twist.covariance[21] = 1e6;
	od.twist.covariance[28] = 1e6;
	od.twist.covariance[35] = 1e-9;

	pub.publish(od);

	last_time = current_time;

}
	
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "odom_encoder");

	ros::NodeHandle n;
	//ros::NodeHandle nh("~");

	pub = n.advertise<nav_msgs::Odometry>("odom", 100);
	
	ros::Subscriber sub = n.subscribe("encoder", 1000, encoder_Callback);
	//tf::TransformBroadcaster od_brod;
	
	ros::Rate loop_rate(5000);
	int pub_cnt = 0;

	ROS_INFO("Odom by encoder OK!!!");
	ros::spin();
	return 0;
}

