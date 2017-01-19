#include <iostream>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <motoman_viz_msgs/EuclideanLinkTrajectory.h>
#include <tf_conversions/tf_eigen.h>

class FKsolver
{
public:
  FKsolver(ros::NodeHandle &nh);
private:
  void DisplayTrajectoryCB(const moveit_msgs::DisplayTrajectory::ConstPtr& display_traj);

  ros::Publisher euc_link_traj_pub;
  ros::Subscriber display_traj_sub;

  std::string robot_description_name;
  std::string model_group_name;
};

FKsolver::FKsolver(ros::NodeHandle &nh):
  robot_description_name("robot_description"),
  model_group_name("arm")
{
  ros::NodeHandle n("~");

  n.param("motoman_viz/robot_description_name", robot_description_name, robot_description_name);
  n.param("motoman_viz/model_group_name", model_group_name, model_group_name);

  euc_link_traj_pub = nh.advertise<motoman_viz_msgs::EuclideanLinkTrajectory>("/move_group/display_planned_euclidean_path",1);
  display_traj_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1, &FKsolver::DisplayTrajectoryCB, this);

  ROS_INFO("Motoman Viz : Initialized");
}

void FKsolver::DisplayTrajectoryCB(const moveit_msgs::DisplayTrajectory::ConstPtr& display_traj)
{
  // Initialization of the Kinematic Model (MoveIt!)
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description_name);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(model_group_name);
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  const std::vector<std::string> &link_names = joint_model_group->getLinkModelNames();
  
  // Initialization of the Euclidean Link Trajectory Message
  motoman_viz_msgs::EuclideanLinkTrajectory euc_link_traj;
  euc_link_traj.header = display_traj->trajectory[0].joint_trajectory.header;
  for(int i=0; i<link_names.size(); i++)
	euc_link_traj.link_names.push_back(link_names[i]);
  euc_link_traj.points.resize(display_traj->trajectory[0].joint_trajectory.points.size());
  for(int i=0; i<euc_link_traj.points.size(); i++)
	euc_link_traj.points[i].pose.resize(link_names.size());
   
  

  // Convert the joint configuration to euclidedan information
  for(int i=0; i<euc_link_traj.points.size(); i++){
	// copy the time from start
	euc_link_traj.points[i].time_from_start = display_traj->trajectory[0].joint_trajectory.points[i].time_from_start;

	// set the joint positions
	std::vector<double> joint_positions;
	for(int j=0; j<joint_names.size(); j++)
	  joint_positions.push_back(display_traj->trajectory[0].joint_trajectory.points[i].positions[j]);
	kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
	
	// convert from joint configuration to euclidean information
	for(int j=0; j<link_names.size(); j++){
	  const Eigen::Affine3d &euc_link_state = kinematic_state->getGlobalLinkTransform(link_names[j]);
	  tf::Pose transform;
	  tf::poseEigenToTF(euc_link_state, transform);
	  tf::Vector3 origin = transform.getOrigin();
	  tf::Quaternion q = transform.getRotation();
	  q.normalize();
	  euc_link_traj.points[i].pose[j].position.x = origin.x();
	  euc_link_traj.points[i].pose[j].position.y = origin.y();
	  euc_link_traj.points[i].pose[j].position.z = origin.z();
	  euc_link_traj.points[i].pose[j].orientation.x = q.x();
	  euc_link_traj.points[i].pose[j].orientation.y = q.y();
	  euc_link_traj.points[i].pose[j].orientation.z = q.z();
	  euc_link_traj.points[i].pose[j].orientation.w = q.w();
	}
  }
  euc_link_traj_pub.publish(euc_link_traj);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forward_kinematics_solver_node");
  ros::NodeHandle n;
  FKsolver fk_solver(n);
  ros::spin();

  return 0;
}
