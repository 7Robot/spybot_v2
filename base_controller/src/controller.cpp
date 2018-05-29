#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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

class Base
{
private:
	int PI;
	ros::NodeHandle m_n;

	float m_target_x;
	float m_target_th;
	float m_current_x;
	float m_current_th;

	float m_sum_err_x;
	float m_sum_err_th;

	float m_err_pre_x;
	float m_err_pre_th;

	float m_kp_x;
	float m_ki_x;
	float m_kd_x;

	float m_kp_th;
	float m_ki_th;
	float m_kd_th;

	int m_t_pid;

	void set_speedLeft(int speed);
	void set_speedRight(int speed);

	void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void odo_callback(const nav_msgs::Odometry::ConstPtr& odom);
	void pid_callback(const ros::TimerEvent& trash);

public:
	Base();
	~Base();
};

Base::Base()
{
	m_n.param("period", m_t_pid, 0.01);
	m_n.param("kp_x", m_kp_x, 1);
	m_n.param("kd_x", m_kd_x, 0);
	m_n.param("ki_x", m_ki_x, 0);
	m_n.param("kp_th", m_kp_th, 1);
	m_n.param("kd_th", m_kd_th, 0);
	m_n.param("ki_th", m_ki_th, 0);

	PI = pigpio_start(NULL, NULL);

	set_mode(PI, PIN_IN_1, PI_OUTPUT);
	set_mode(PI, PIN_IN_2, PI_OUTPUT);
	set_mode(PI, PIN_IN_3, PI_OUTPUT);
	set_mode(PI, PIN_IN_4, PI_OUTPUT);

	ros::Subscriber cmd_sub = m_n.subscribe("cmd_vel", 100, cmd_callback);
	ros::Subscriber odo_sub = m_n.subscribe("odom", 100, odo_callback);
	ros::Timer timer = m_n.createTimer(ros::Duration(m_t_pid), pid_callback);
}

Base::~Base()
{
	set_speedLeft(0);
	set_speedRight(0);

	pigpio_stop(PI);
}

void Base::set_speedLeft(int speed)
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

void Base::set_speedRight(int speed)
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

void Base::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	m_target_x = cmd_vel->linear.x;
	m_target_th = cmd_vel->angular.z;
}

void Base::odo_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	m_current_x = odom->twist.twist.linear.x;
	m_current_th = odom->twist.twist.angular.z;
}

void Base::pid_callback(const ros::TimerEvent& trash)
{
	float err_x, err_th, der_x, der_th, pid_x, pid_th;
	int speed_left, speed_right;

	//Calculate errors
	err_x = m_target_x - m_current_x;
	err_th = m_target_th - m_current_th;

	//Calculate error sums
	m_sum_err_x += err_x;
	if(m_sum_err_x > 200)
		m_sum_err_x = 200;
	if(m_sum_err_x < -200)
		m_sum_err_x = -200;


	m_sum_err_th += err_th;
	if(m_sum_err_x > 200)
		m_sum_err_x = 200;
	if(m_sum_err_x < -200)
		m_sum_err_x = -200;

	//Calculate error derivates
	der_x = (err_x - m_err_pre_x);
	der_th = (err_th - m_err_pre_th);

	//Calculate pid
	pid_x = kp_x * err_x + kd_x * der_x + ki_x * m_sum_err_x;
	pid_th = kp_th * err_x + kd_th * der_x + ki_th * m_sum_err_x;

	speed_right = pid_x + pid_th;
	if(speed_right > 250)
		speed_right = 250;
	if(speed_right < -250)
		speed_right = -250;

	speed_left = pid_x - pid_th;
	if(speed_left > 250)
		speed_left = 250;
	if(speed_left < -250)
		speed_left = -250;


	set_speed_left(speed_left);
	set_speed_right(speed_right);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_controller");

	Base base();

	ros::spin();

	return 0;
}
