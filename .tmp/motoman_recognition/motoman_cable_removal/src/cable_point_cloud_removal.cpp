// #include <cable_point_cloud_removal.hpp>
#include "../include/cable_point_cloud_removal.hpp"
using namespace pcl;

CableRemove::CableRemove(ros::NodeHandle nh, ros::NodeHandle n)
    : nh_(nh),
      rate_(n.param("loop_rate", 10)),
      frame_id_(n.param<std::string>("frame_id", "world"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &CableRemove::CableRemoveCallback, this);
  fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("filtered_pc_topic_name", "/cable_removed_pointcloud"), 1);

  // clopboxを当てはめるエリアを定義
  n.param<float>("crop_x_min", crop_min_.x, -0.6);
  n.param<float>("crop_x_max", crop_max_.x, 0.6);
  n.param<float>("crop_y_min", crop_min_.y, -0.8);
  n.param<float>("crop_y_max", crop_max_.y, 0.8);
  n.param<float>("crop_z_min", crop_min_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_.z, 2.5);

  nh.param<double>("cable_start_pos_x", cable_start_pos_[0], 0.36);
  nh.param<double>("cable_start_pos_y", cable_start_pos_[1], 0.00);
  nh.param<double>("cable_start_pos_z", cable_start_pos_[2], 1.56);
  // ケーブルの消す範囲（ケーブルを軸としたときの半径）
  nh.param<double>("radius_threshold", radius_threshold_, 0.1);
}

void CableRemove::CableRemoveCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {
  try{
    tf_.lookupTransform("/world", "/dhand_base_link", source_pc->header.stamp, transform_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // std::cout << "(x, y, z) = ("
  //           << transform.getOrigin().x()
  //           << ", "
  //           << transform.getOrigin().y()
  //           << ", "
  //           << transform.getOrigin().z()
  //           << ")"
  //           << std::endl;

  dhand_adapter_pos_[0] = transform_.getOrigin().x();
  dhand_adapter_pos_[1] = transform_.getOrigin().y();
  dhand_adapter_pos_[2] = transform_.getOrigin().z();

  sensor_msgs::PointCloud2 trans_pc;
  try {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));

  CableRemove::CropBox(pcl_source_ptr, crop_min_, crop_max_);
  for(pcl::PointCloud<PointXYZ>::iterator pcl_source_ptr_i = pcl_source_ptr->points.begin(); pcl_source_ptr_i < pcl_source_ptr->points.end(); ++pcl_source_ptr_i){
    if (pcl_source_ptr_i->z > dhand_adapter_pos_[2]){
      Eigen::Vector3f AB(dhand_adapter_pos_[0] - cable_start_pos_[0],
                         dhand_adapter_pos_[1] - cable_start_pos_[1],
                         dhand_adapter_pos_[2] - cable_start_pos_[2]);
      Eigen::Vector3f AP(pcl_source_ptr_i->x - cable_start_pos_[0],
                         pcl_source_ptr_i->y - cable_start_pos_[1],
                         pcl_source_ptr_i->z - cable_start_pos_[2]);
      Eigen::Vector3f ABcrossAP = AB.cross(AP);
      float Length_AB = AB.norm();
      float Length_ABCrossAP = ABcrossAP.norm();
      float H = Length_ABCrossAP / Length_AB;
      if( H < radius_threshold_ ){
        pcl_source_ptr->erase(pcl_source_ptr_i);
        pcl_source_ptr_i--;
        // ROS_INFO("Remove Cable Point Cloud H = %f", H);
      }
    }
  }

  // 処理後の点群をpublish
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  filtered_pc2.header.stamp = source_pc->header.stamp;
  filtered_pc2.header.frame_id = frame_id_;
  fileterd_cloud_pub_.publish(filtered_pc2);
  ROS_INFO("Cable remove point cloud published");
}

void CableRemove::CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PointXYZ min, pcl::PointXYZ max) {
  Eigen::Vector4f minPoint;

  minPoint[0] = min.x; // define minimum point x
  minPoint[1] = min.y; // define minimum point y
  minPoint[2] = min.z; // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = max.x; // define max point x
  maxPoint[1] = max.y; // define max point y
  maxPoint[2] = max.z; // define max point z

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0; // rotation around x-axis
  boxRotation[1] = 0; // rotation around y-axis
  boxRotation[2] = 0; // in radians rotation around z-axis. this rotates your

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud_filtered, *cloud);
}

void CableRemove::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
