#ifndef CABLE_POINT_CLOUD_REMOVAL_H
#define CABLE_POINT_CLOUD_REMOVAL_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/impl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <visualization_msgs/Marker.h>

using namespace pcl;

class CableRemove {
public:
  CableRemove(ros::NodeHandle nh, ros::NodeHandle n);
  void CableRemoveCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc);
  void CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  std::string transformed_frame_id_;
  ros::Publisher fileterd_cloud_pub_;

  ros::Subscriber source_pc_sub_;
  tf::TransformListener tf_;
  tf::StampedTransform transform_;

  Eigen::Vector3d dhand_adapter_pos_;
  Eigen::Vector3d cable_start_pos_;

  pcl::PointXYZ crop_min_, crop_max_;

  double radius_threshold_;

  double leaf_size_x_;
  double leaf_size_y_;
  double leaf_size_z_;
};

#endif /* CABLE_POINT_CLOUD_REMOVAL_H */
