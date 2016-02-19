#include <ros.h> // Use ros_lib.
#include <std_msgs/Int32.h>

void switchCb(const std_msgs::Int32& msg); 

/* Declare global variables. */
ros::NodeHandle nh; // The nodeHandle.
ros::Subscriber<std_msgs::Int32> sub("/jamming_grasp", &switchCb); // Set subscribe the motor_driver topic.

void setup() {
  /* Set pins Mode. */
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  /* Node handle setting. */
  nh.initNode(); // First setup the node handle.
  nh.subscribe(sub); // Start subscribe the "steer_ctrl" topic.
}

void loop() {
  nh.spinOnce(); // Check topic and if change it, run the call back function.
}


void switchCb(const std_msgs::Int32& msg) {
  if(msg.data == 1){
	digitalWrite(8, HIGH);
  }
  else{
	digitalWrite(8, LOW);
  }
}
