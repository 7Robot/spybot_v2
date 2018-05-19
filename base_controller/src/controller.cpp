#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pigpio.h>
#include <cstdio>
#include <cmath>

#define PIN_EN_A 14
#define PIN_IN_1 15
#define PIN_IN_2 18

#define PIN_EN_B 17
#define PIN_IN_3 27
#define PIN_IN_4 22

#define DIST_ENTRAXE 0.25

using namespace std;

float speed_target_r;
float speed_target_l;

void set_speed_left(int speed)
{
	if(speed > 0)
	{
		gpioWrite(PIN_IN_1, 1);
		gpioWrite(PIN_IN_2, 0);
		gpioPWM(PIN_EN_A, speed);
	}
	else
	{
		gpioWrite(PIN_IN_1, 0);
		gpioWrite(PIN_IN_2, 1);
		gpioPWM(PIN_EN_A, -speed);
	}
}

void set_speed_right(int speed)
{
	if(speed > 0)
	{
		gpioWrite(PIN_IN_3, 0);
		gpioWrite(PIN_IN_4, 1);
		gpioPWM(PIN_EN_B,speed);
	}
	else
	{
		gpioWrite(PIN_IN_3, 1);
		gpioWrite(PIN_IN_4, 0);
		gpioPWM(PIN_EN_B, -speed);
	}
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
		speed_target_r = cmd_vel->linear.x + 200 * DIST_ENTRAXE * tan(cmd_vel->angular.z);
		speed_target_l = cmd_vel->linear.x - 200 * DIST_ENTRAXE * tan(cmd_vel->angular.z);
}

void reg_callback(const ros::TimerEvent& trash)
{
	set_speed_left((int)speed_target_l);
	set_speed_right((int)speed_target_r);
}

int main(int argc, char** argv)
{
	int speed;
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;

	if(gpioInitialise() < 0)
		return -1;

	gpioSetMode(PIN_IN_1, PI_OUTPUT);
	gpioSetMode(PIN_IN_2, PI_OUTPUT);
	gpioSetMode(PIN_IN_3, PI_OUTPUT);
	gpioSetMode(PIN_IN_4, PI_OUTPUT);

	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_callback);
	ros::Timer timer = n.createTimer(ros::Duration(0.01), reg_callback);

	ros::spin();

	set_speed_left(0);
	set_speed_right(0);

	gpioTerminate();

	return 0;
}
