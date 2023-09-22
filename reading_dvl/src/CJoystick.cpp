#include "CJoystick.h"

CJoystick::CJoystick(){a=b=y=x=false,JYaw=JGaz=JRoll=JPitch=0;}

CJoystick::~CJoystick(){}

void CJoystick::chatterCallback(const sensor_msgs::Joy& msg)
{
  a = msg.buttons[0]; 
  b = msg.buttons[1]; 
  y = msg.buttons[3]; 
  x = msg.buttons[2]; 
  RB= msg.buttons[5]; 
  b5 = msg.buttons[4];
  b6 = msg.buttons[5];
  b7 = msg.buttons[6];
  b8 = msg.buttons[7];

  this-> JYaw = msg.axes[0]; 
  this-> JGaz = msg.axes[1]; 
  this-> JRoll = msg.axes[3]; 
  this-> JPitch = msg.axes[4];
}

void CJoystick::publishMessage(ros::Publisher *const H)
{
	std_msgs::Empty msg;
	H->publish(msg);

}


