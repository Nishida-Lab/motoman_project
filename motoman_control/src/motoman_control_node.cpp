#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#define PI 3.14159265

class SIA5Arm
{
protected:
  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  
  trajectory_msgs::JointTrajectory goal_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;

  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint current_pt;
  int traj_pt_num;
  int traj_pt_cnt;
  
  ros::Subscriber sub_pose_arm_;
  ros::Publisher pub_move_arm_;

public:

  SIA5Arm(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
	traj_pt_num(0),
	traj_pt_cnt(0)
  {
    as_.registerGoalCallback(boost::bind(&SIA5Arm::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&SIA5Arm::preemptCB, this));

    sub_pose_arm_ = nh_.subscribe("/joint_states", 1, &SIA5Arm::analysisCB, this);

    pub_move_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1, this);

    as_.start();
  }

  ~SIA5Arm(void)
  {
  }

  void goalCB()
  {
	goal_ = as_.acceptNewGoal()->trajectory;
	traj_pt_num = goal_.points.size();
	ROS_INFO("Goal Recieived");
	pub_move_arm_.publish(goal_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::JointState::ConstPtr &joint_state)
  {	
    if (!as_.isActive())
      return;

	// int joint_num = sizeof(joint_state->name)/sizeof(joint_state->name[0]);
	int joint_num = 7;

	
    feedback_.header = joint_state->header;
    feedback_.joint_names.resize(joint_num);
    feedback_.actual.positions.resize(joint_num);
    feedback_.actual.effort.resize(joint_num);
	
	for(int i=0; i<joint_num; i++){
	  feedback_.joint_names[i] = joint_state->name[i];
	  feedback_.actual.positions[i] = joint_state->position[i];
	}
	 
	// feedback に desired と error を入れる---
	feedback_.desired = goal_.points[traj_pt_cnt];
	feedback_.error.positions.resize(joint_num);
	// feedbackをpublish
	as_.publishFeedback(feedback_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sia5_controller/follow_joint_trajectory");

  SIA5Arm sia5arm(ros::this_node::getName());
  ros::spin();

  return 0;
}
