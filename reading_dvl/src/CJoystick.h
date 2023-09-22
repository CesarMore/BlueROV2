#ifndef CJoystick_H
#define CJoystick_H

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

using std::string;

class CJoystick
{
public:
	CJoystick();
	~CJoystick();
	void chatterCallback(const sensor_msgs::Joy& msg);
	void publishMessage(ros::Publisher *const); 

    float JYaw;
	bool a;
	bool b;
	bool y;
	bool x;
	bool RB;
        bool b5;
        bool b6;
        bool b7;
        bool b8;
    float JGaz;
    float JRoll;
    float JPitch;
 };
#endif

