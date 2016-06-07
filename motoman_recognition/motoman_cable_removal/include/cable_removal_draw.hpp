#ifndef CABLE_REMOVAL_DRAW_H
#define CABLE_REMOVAL_DRAW_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/filters/crop_box.h>
#include <pcl_ros/impl/transforms.hpp>

#include <visualization_msgs/Marker.h>

using namespace pcl;

class CableDraw {
public:
  CableDraw(ros::NodeHandle nh, ros::NodeHandle n);
  void DrawLoop();
  void DrawCable(Eigen::Vector3d a, Eigen::Vector3d b);
  void DrawCylinder(Eigen::Vector3d a, Eigen::Vector3d b, double length);
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher cable_line_pub_;
  ros::Publisher remove_area_pub_;
  tf::TransformListener tf_;
  tf::StampedTransform transform_;

  Eigen::Vector3d dhand_adapter_pos_;
  Eigen::Vector3d cable_start_pos_;

  double radius_threshold_;
};

#endif /* CABLE_REMOVAL_DRAW_H */
