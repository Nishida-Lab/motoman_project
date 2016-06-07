// #include <cable_removal_draw.hpp>
#include "../include/cable_removal_draw.hpp"
using namespace pcl;

CableDraw::CableDraw(ros::NodeHandle nh, ros::NodeHandle n)
    : nh_(nh),
      rate_(n.param("loop_rate", 10)),
      frame_id_(n.param<std::string>("frame_id", "world"))
{
  cable_line_pub_ = nh_.advertise<visualization_msgs::Marker>(n.param<std::string>("cable_name", "/cable"), 1);
  remove_area_pub_ = nh_.advertise<visualization_msgs::Marker>(n.param<std::string>("remove_area_name", "/remove_area"), 1);

  nh.param<double>("cable_start_pos_x", cable_start_pos_[0], 0.36);
  nh.param<double>("cable_start_pos_y", cable_start_pos_[1], 0.00);
  nh.param<double>("cable_start_pos_z", cable_start_pos_[2], 1.56);
  // ケーブルの消す範囲（ケーブルを軸としたときの半径）
  nh.param<double>("radius_threshold", radius_threshold_, 0.1);
}

void CableDraw::DrawLoop() {
  try{
    tf_.lookupTransform("/world", "/dhand_base_link", ros::Time(0), transform_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  dhand_adapter_pos_[0] = transform_.getOrigin().x();
  dhand_adapter_pos_[1] = transform_.getOrigin().y();
  dhand_adapter_pos_[2] = transform_.getOrigin().z();

  DrawCable(cable_start_pos_, dhand_adapter_pos_);
  double length = (dhand_adapter_pos_ - cable_start_pos_).norm() - 0.15;
  DrawCylinder(cable_start_pos_, dhand_adapter_pos_, length);
}

void CableDraw::DrawCable(Eigen::Vector3d a, Eigen::Vector3d b)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id_;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.02;

  line_list.color.r = 0.0;
  line_list.color.g = 0.0;
  line_list.color.b = 0.0;
  line_list.color.a = 0.5;

  geometry_msgs::Point p;
  p.x = a[0];
  p.y = a[1];
  p.z = a[2];

  line_list.points.push_back(p);

  p.x = b[0];
  p.y = b[1];
  p.z = b[2];

  line_list.points.push_back(p);

  cable_line_pub_.publish(line_list);
}

void CableDraw::DrawCylinder(Eigen::Vector3d a, Eigen::Vector3d b, double length)
{
  Eigen::Vector3d axis_vector = b - a;
  Eigen::Vector3d axis_vector_normalized = axis_vector.normalized();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector_normalized.cross(up_vector);
  right_axis_vector.normalize();

  double theta = axis_vector_normalized.dot(up_vector);
  double angle_rotation = -acos(theta);

  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  Eigen::Quaterniond q;
  tf::quaternionTFToEigen(tf_q, q);

  // Rotate so that vector points along line
  q.normalize();

  // Draw marker
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = (a[0] + b[0])/2.0;
  marker.pose.position.y = (a[1] + b[1])/2.0;
  marker.pose.position.z = (a[2] + b[2])/2.0;
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 2*radius_threshold_;
  marker.scale.y = 2*radius_threshold_;
  marker.scale.z = length;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 231.0/255.0;
  marker.color.g = 79.0/255.0;
  marker.color.b = 81.0/255.0;
  marker.color.a = 0.25;

  marker.lifetime = ros::Duration();

  // Publish the marker
  remove_area_pub_.publish(marker);
}

void CableDraw::run()
{
  while(nh_.ok()){
    DrawLoop();
    ros::spinOnce();
    rate_.sleep();
  }
}
