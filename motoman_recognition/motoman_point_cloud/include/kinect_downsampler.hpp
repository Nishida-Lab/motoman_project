#ifndef KINECT_DOWNSAMPLER_H
#define KINECT_DOWNSAMPLER_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

class KinectV2Downsampler
{
public:
  KinectV2Downsampler(ros::NodeHandle nh, ros::NodeHandle n);
  void resizeCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void run();
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher resized_pc_pub_;
  ros::Subscriber source_pc_sub_;

  double leaf_size_x_;
  double leaf_size_y_;
  double leaf_size_z_;
};

#endif /* KINECT_DOWNSAMPLER_H */
