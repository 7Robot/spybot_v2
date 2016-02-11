#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/ioctl.h>
#include <string>
#include <termios.h>

using namespace std;

int keypressed()
{
	static const int STDIN = 0;
	static bool initialized = false;
	termios term;
	int bytesWaiting;

	if(!initialized)
	{
		tcgetattr(STDIN, &term);
		term.c_lflag &= ~ICANON;
		tcsetattr(STDIN, TCSANOW, &term);
		setbuf(stdin, NULL);

		cin.sync_with_stdio();

		initialized = true;
	}

	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

int main(int argc, char* argv[])
{
	char c;
  double linear_speed = 0;
  double angular_speed = 0;

  geometry_msgs::Twist vel;

  ros::init(argc, argv, "teleop_key");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate r(100);

	//Background task
	while(ros::ok())
	{
    linear_speed = 0;
    angular_speed = 0;

		if(keypressed())
		{
			c = getchar();
			//directionnal arrow
			if(c == '\x1b')
			{
				c = getchar();
				c = getchar();

				switch(c)
				{
					//UP A
					case 'A':
						linear_speed = 1;
						break;
					//DOWN B
					case 'B':
            linear_speed = -1;
						break;
					//RIGHT C
					case 'C':
            angular_speed = 1;
						break;
					//LEFT D
					case 'D':
            angular_speed = -1;
						break;
				}
			}
    }

    vel.angular.z = angular_speed;
    vel.linear.x = linear_speed;

    pub.publish(vel);

    r.sleep();

	}

  return 0;
}
