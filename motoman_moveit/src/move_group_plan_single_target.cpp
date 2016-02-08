#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "move_group_plan_single_target");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // Get the arm planning group
    moveit::planning_interface::MoveGroup plan_group("arm");

    // Create a published for the arm plan visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // Set a goal message as a pose of the end effector
    geometry_msgs::Pose goal;
    goal.orientation.x = 0;
    goal.orientation.y = 0;
    goal.orientation.z = 0;
    goal.orientation.w = 1;
    goal.position.x = 0.5;
    goal.position.y = 0.3;
    goal.position.z = 0.5;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }

    sleep(7.0);

    goal.orientation.x = 0;
    goal.orientation.y = 0;
    goal.orientation.z = 0;
    goal.orientation.w = 1;
    goal.position.x = 0.5;
    goal.position.y = -0.3;
    goal.position.z = 0.5;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan2;
    if (plan_group.plan(goal_plan2))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan2.start_state_;
        display_msg.trajectory.push_back(goal_plan2.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    } 
    ros::shutdown();

    return 0;
}
