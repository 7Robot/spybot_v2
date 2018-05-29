#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pigpiod_if2.h>
#include <cstdio>
#include <cmath>

#define PIN_EN_A 15
#define PIN_IN_1 18
#define PIN_IN_2 23

#define PIN_EN_B 17
#define PIN_IN_3 27
#define PIN_IN_4 22

#define DIST_ENTRAXE 0.3

using namespace std;

int PI;

float speed_target_r;
float speed_target_l;

void set_speed_left(int speed)
{
	if(speed > 0)
	{
		gpio_write(PI, PIN_IN_1, 1);
		gpio_write(PI, PIN_IN_2, 0);
		set_PWM_dutycycle(PI, PIN_EN_A, speed);
	}
	else
	{
		gpio_write(PI, PIN_IN_1, 0);
		gpio_write(PI, PIN_IN_2, 1);
		set_PWM_dutycycle(PI, PIN_EN_A, -speed);
	}
}

void set_speed_right(int speed)
{
	if(speed > 0)
	{
		gpio_write(PI, PIN_IN_3, 0);
		gpio_write(PI, PIN_IN_4, 1);
		set_PWM_dutycycle(PI, PIN_EN_B,speed);
	}
	else
	{
		gpio_write(PI, PIN_IN_3, 1);
		gpio_write(PI, PIN_IN_4, 0);
		set_PWM_dutycycle(PI, PIN_EN_B,-speed);
	}
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
		speed_target_r = 250*cmd_vel->linear.x + 250*cmd_vel->angular.z;
		speed_target_l = 250*cmd_vel->linear.x - 250*cmd_vel->angular.z;
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

	PI = pigpio_start(NULL, NULL);

	set_mode(PI, PIN_IN_1, PI_OUTPUT);
	set_mode(PI, PIN_IN_2, PI_OUTPUT);
	set_mode(PI, PIN_IN_3, PI_OUTPUT);
	set_mode(PI, PIN_IN_4, PI_OUTPUT);
	set_PWM_frequency(PI, PIN_EN_A, 100000);
	set_PWM_frequency(PI, PIN_EN_B, 100000);

	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_callback);
	ros::Timer timer = n.createTimer(ros::Duration(0.01), reg_callback);

	ros::spin();

	set_speed_left(0);
	set_speed_right(0);

	pigpio_stop(PI);

	return 0;
}
