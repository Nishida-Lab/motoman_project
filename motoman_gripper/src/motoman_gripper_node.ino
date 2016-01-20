#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle n;
std_msgs::Bool data;  
int hang_pin = 8;

ros::Subscriber<std_msgs::Bool> sub("sia5/gripper", &hangCallback);

// Call Back
void hangCallback(const std_msgs::Bool &hang_on){
  if(hang_on.data)
	digitalWrite(hang_pin, HIGH);
  else
	digitalWrite(hang_pin, LOW);
}

void setup()
{
  pinMode(hang_pin, OUTPUT);
  n.initNode();
  node.Subscriber(sub);
}

void loop()
{
  n.spinOnce();
}
