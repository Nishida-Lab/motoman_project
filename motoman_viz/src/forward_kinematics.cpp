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
#include <motoman_viz_msgs/EuclideanJointTrajectory.h>

class FKsolver
{
public:
  FKsolver(ros::NodeHandle &nh);
private:
  void DisplayTrajectoryCB(const moveit_msgs::DisplayTrajectory::ConstPtr& display_traj);

  ros::Publisher euc_joint_traj_pub;
  ros::Subscriber display_traj_sub;

  std::string robot_description_name;
};

FKsolver::FKsolver(ros::NodeHandle &nh):
  robot_description_name("robot_description")
{
  ros::NodeHandle n("~");

  n.param("motoman_viz/robot_description_name",robot_description_name, robot_description_name);

  euc_joint_traj_pub = nh.advertise<motoman_viz_msgs::EuclideanJointTrajectory>("/move_group/display_planned_euclidean_path",1);
  display_traj_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1, &FKsolver::DisplayTrajectoryCB, this);
}

void FKsolver::DisplayTrajectoryCB(const moveit_msgs::DisplayTrajectory::ConstPtr& display_traj)
{
  // Initialization of the Kinematic Model
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description_name);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  std::cout << "Initialized Moveit" << std::endl;
  
  // Initialization of the Euclidean Joint Trajectory Message
  motoman_viz_msgs::EuclideanJointTrajectory euc_joint_traj;
  euc_joint_traj.header = display_traj->trajectory[0].joint_trajectory.header;
  //std::map<std::string, double> euc_joint_state;
  for(int i=0; i<display_traj->trajectory[0].joint_trajectory.joint_names.size(); i++){
	euc_joint_traj.joint_names.push_back(display_traj->trajectory[0].joint_trajectory.joint_names.at(i));
	//euc_joint_state.insert(std::pair<std::string, double>(display_traj->trajectory[0].joint_trajectory.joint_names.at(i),0.0));
  }

  std::cout << "Initialized Euclidean Joint Trajectory Message" << std::endl;
  
  std::vector< std::map<std::string, double> > euc_joint_state_arry;
  for(int i=0; i<display_traj->trajectory[0].joint_trajectory.points.size(); i++){
	for(int j=0; j<display_traj->trajectory[0].joint_trajectory.joint_names.size(); j++)
	  euc_joint_state[display_traj->trajectory[0].joint_trajectory.joint_names.at(j)] = display_traj->trajectory[0].joint_trajectory.points[i].positions[j];
	euc_joint_state_arry.push_back(euc_joint_state);
  }

  std::cout << "Initialized Map datas" << std::endl;

  for(int i=0; i<euc_joint_state_arry.size(); i++){
	//kinematics_state->setJointPositions(euc_joint_traj.joint_names)
	//kinematic_state->setVariablePositions(euc_joint_state_arry[i]);
	for(int j=0; j<euc_joint_state_arry[i].size(); j++){
	  std::cout << "Get Euclidean data" << std::cout;
	  const Eigen::Affine3d& euc_frame_state = kinematic_state->getGlobalLinkTransform(euc_joint_traj.joint_names[j]);
	  euc_frame_state.linear();
	  std::cout << "Homogeneous Matrix: " << euc_frame_state.matrix() << std::endl;
	}
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forward_kinematics_solver_node");
  ros::NodeHandle n;
  FKsolver fk_solver(n);
  ros::spin();

  return 0;
}
