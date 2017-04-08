
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <map>

class SIA5Arm
{
protected:
  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  
  trajectory_msgs::JointTrajectory goal_;

  ros::Subscriber sub_joint_state;
  ros::Publisher pub_move_arm_;

public:

  SIA5Arm(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    as_.registerGoalCallback(boost::bind(&SIA5Arm::goalCB, this));

	sub_joint_state = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &SIA5Arm::JointStateCallback, this);
	
    pub_move_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1, this);

	ros::Time::init();
	
    as_.start();
  }

  void goalCB()
  {
	goal_ = as_.acceptNewGoal()->trajectory;
	for(int i=0; i<goal_.joint_names.size(); i++)
	  goal_.points[0].positions[i] = js_map_[goal_.joint_names[i]];
	ROS_INFO("Goal Recieived");
	pub_move_arm_.publish(goal_);
	ROS_INFO("Moving...");
	ros::Duration tfs = goal_.points[goal_.points.size()-1].time_from_start;
	ROS_INFO("Task Done !!");	
  }


private:
  std::map<std::string, double> js_map_;
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& js)
  {
	for(int i=0; i<js->name.size(); i++)
	  js_map_[js->name[i]] = js->position[i];
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sia5_controller/follow_joint_trajectory");

  SIA5Arm sia5arm(ros::this_node::getName());
  ros::spin();

  return 0;
}
