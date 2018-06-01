#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pigpiod_if2.h>
#include <cstdio>
#include <cmath>

#include "rotary_encoder.hpp"

#define PI 3.1415
#define STEP_TO_M PI/1024*0.058
//TODO verifier la dist entraxe
#define DIST_ENTRAXE 0.29
#define FREQ 10

// Rotary encoder on 10 bits [0; 1023]

using namespace std;
static double pos_l = 0;
static double pos_r = 0;

void callback_left(int way)
{

	pos_l += way*STEP_TO_M;
	//cout << "position gauche : " << pos_l << endl;
}

void callback_right(int way)
{
	pos_r += way*STEP_TO_M;
	//cout << "position droite : " << pos_r << endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odometry");
	ros::NodeHandle n;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	double p_pos_l = 0;
	double p_pos_r = 0;

	double delta_l;
	double delta_r;

	double vl;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;

	int _PI;

	_PI = pigpio_start(NULL, NULL);

	re_decoder left_dec(_PI, 20, 26, callback_left);
	re_decoder right_dec(_PI, 19,16, callback_right);


	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(FREQ);

	while(n.ok()){

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		delta_l = pos_l - p_pos_l;
		delta_r = pos_r - p_pos_r;

		p_pos_l = pos_l;
		p_pos_r = pos_r;

    double delta_s = (delta_r + delta_l) * 0.5; // on peut faire du bit shift si tu prefere
		double delta_th = (delta_r - delta_l) / DIST_ENTRAXE;

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = delta_s * cos(th + (delta_th / 2));
		double delta_y = delta_s * sin(th + (delta_th / 2));

		x += delta_x;
		y += delta_y;
		th += delta_th;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = delta_x / dt;
		odom.twist.twist.linear.y = delta_y / dt;
		odom.twist.twist.angular.z = delta_th / dt;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		//cout << "x:" << x << ", y:" << y <<", th:" << th << ", vl:" << delta_s/dt << ", vth:" << delta_th/dt << endl;
		r.sleep();
	}

	left_dec.re_cancel();
	right_dec.re_cancel();

	pigpio_stop(_PI);

	return 0;
}
