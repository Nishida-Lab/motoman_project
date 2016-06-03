#ifndef CABLE_POINT_CLOUD_REMOVAL_H
#define CABLE_POINT_CLOUD_REMOVAL_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <visualization_msgs/Marker.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>

using namespace pcl;

class CableRemove {
public:
  CableRemove(ros::NodeHandle nh, ros::NodeHandle n);
  void CableRemoveCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc);
  void CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);
  void DrawCable(Eigen::Vector3d a, Eigen::Vector3d b);
  Eigen::Affine3f DrawBox(Eigen::Vector3d a, Eigen::Vector3d b, double length);
  void CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Affine3f matrix, double length);
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher fileterd_cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher bounding_pub_;
  ros::Subscriber source_pc_sub_;
  tf::TransformListener tf_;
  // tf::TransformBroadcaster br_;
};

#endif /* CABLE_POINT_CLOUD_REMOVAL_H */
