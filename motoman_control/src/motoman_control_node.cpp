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
  trajectory_msgs::JointTrajectoryPoint traj_pt[2];
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
	ROS_INFO("Goal Recieived : %d", traj_pt_num);
	// ROS_INFO("Goal Received : [%lf,%lf,%lf,%lf,%lf,%lf,%lf]", goal_.points[4].velocities[0],goal_.points[4].velocities[1],goal_.points[4].velocities[2],goal_.points[4].velocities[3],goal_.points[4].velocities[4],goal_.points[4].velocities[5],goal_.points[4].velocities[6]);
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

	bool reach_traj_pt = true;


	// Get feed back datas ----
	// int joint_num = sizeof(joint_state->name)/sizeof(joint_state->name[0]);
	int joint_num = 7;
	// ROS_INFO("joint_num : %d", joint_num);

	
    feedback_.header = joint_state->header;
    feedback_.joint_names.resize(joint_num);
    feedback_.actual.positions.resize(joint_num);
    feedback_.actual.effort.resize(joint_num);
	
	for(int i=0; i<joint_num; i++){
	  feedback_.joint_names[i] = joint_state->name[i];
	  feedback_.actual.positions[i] = joint_state->position[i];
	}


	traj_pt[0].positions.resize(joint_num);
	traj_pt[0].velocities.resize(joint_num);
	traj_pt[1].positions.resize(joint_num);
	traj_pt[1].velocities.resize(joint_num);

	// Publish JointTrajectory message that means motoman will move ----
	if(traj_pt_cnt < traj_pt_num){	  
	  for(int i=0; i<joint_num; i++){
		traj_pt[0].positions[i] = joint_state->position[i];
		traj_pt[0].velocities[i] = 0;
		traj_pt[1].positions[i] = goal_.points[traj_pt_cnt].positions[i];   // goal position
		traj_pt[1].velocities[i] = goal_.points[traj_pt_cnt].velocities[i]; // robot don't move without velocity data
	  }
	  traj_pt[0].time_from_start = ros::Duration(0.0);
	  traj_pt[1].time_from_start = goal_.points[traj_pt_cnt].time_from_start;
	  
	  // Set goal point to trajectory message
	  traj.header = joint_state->header;
	  traj.joint_names.resize(joint_num);
	  traj.points.resize(2);
	  for(int i=0; i<joint_num; i++)
		traj.joint_names[i] = joint_state->name[i];
	  traj.points[0] = traj_pt[0];
	  traj.points[1] = traj_pt[1];
	  // pub_move_arm_.publish(traj);
	  pub_move_arm_.publish(goal_);
	 
	  // feedback に desired と error を入れる---
	  feedback_.desired = goal_.points[traj_pt_cnt];
	  feedback_.error.positions.resize(joint_num);
	  for (int i = 0; i<joint_num; i++){
		feedback_.error.positions[i] = feedback_.desired.positions[i] - feedback_.actual.positions[i];
		if(fabs(feedback_.error.positions[i]) > 0.04)
		  reach_traj_pt = false;
	  }
	  // feedbackをpublish
	  as_.publishFeedback(feedback_);
	  ROS_INFO("feedback");
	  if(reach_traj_pt)
		ROS_INFO("traj_pt_cnt++ : %d", traj_pt_cnt);
		traj_pt_cnt++;
	} else{
	  as_.setSucceeded();
	}
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sia5_controller/follow_joint_trajectory");

  SIA5Arm sia5arm(ros::this_node::getName());
  ros::spin();

  return 0;
}
