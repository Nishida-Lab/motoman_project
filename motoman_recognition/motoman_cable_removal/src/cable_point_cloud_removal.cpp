// #include <cable_point_cloud_removal.hpp>
#include "../include/cable_point_cloud_removal.hpp"

using namespace pcl;

CableRemove::CableRemove(ros::NodeHandle nh, ros::NodeHandle n)
    : nh_(nh),
      rate_(n.param("loop_rate", 10)),
      frame_id_(n.param<std::string>("clustering_frame_id", "world"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &CableRemove::CableRemoveCallback, this);
  fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("filtered_pc_topic_name", "/cable_removed_pointcloud"), 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // euclidean_cluster_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(n.param<std::string>("box_name", "/clustering_result"), 1);

  // // クラスタリングのパラメータを初期化
  // n.param<double>("clusterTolerance", clusterTolerance_, 0.02);
  // n.param<int>("minSize", minSize_, 100);
  // n.param<int>("maxSize", maxSize_, 25000);
  // // clopboxを当てはめるエリアを定義
  // n.param<float>("crop_x_min", crop_min_.x, 0.15);
  // n.param<float>("crop_x_max", crop_max_.x, 1.5);
  // n.param<float>("crop_y_min", crop_min_.y, -1.5);
  // n.param<float>("crop_y_max", crop_max_.y, 1.5);
  // n.param<float>("crop_z_min", crop_min_.z, 0.01);
  // n.param<float>("crop_z_max", crop_max_.z, 0.5);
}

void CableRemove::CableRemoveCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {
  tf::StampedTransform transform;
  try{
    tf_.lookupTransform("/world", "/dhand_adapter_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  std::cout << "(x, y, z) = ("
            << transform.getOrigin().x()
            << ", "
            << transform.getOrigin().y()
            << ", "
            << transform.getOrigin().z()
            << ")"
            << std::endl;

  Eigen::Vector3f dhand_adapter_pos;
  dhand_adapter_pos[0] = transform.getOrigin().x();
  dhand_adapter_pos[1] = transform.getOrigin().y();
  dhand_adapter_pos[2] = transform.getOrigin().z();
  Eigen::Vector3f A(0.36, 0, 1.56);
  sensor_msgs::PointCloud2 trans_pc;

  DrawCable(A, dhand_adapter_pos);
  try {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));

  for(pcl::PointCloud<PointXYZ>::iterator pcl_source_ptr_i = pcl_source_ptr->begin(); pcl_source_ptr_i < pcl_source_ptr->end(); ++pcl_source_ptr_i){
    if (pcl_source_ptr_i->z > dhand_adapter_pos[2]){
      Eigen::Vector3f AB(dhand_adapter_pos[0]-A[0], dhand_adapter_pos[1]-A[1], dhand_adapter_pos[2]-A[2]);
      Eigen::Vector3f AP(pcl_source_ptr_i->x-A[0], pcl_source_ptr_i->y-A[1], pcl_source_ptr_i->z-A[2]);
      Eigen::Vector3f ABcrossAP = AB.cross(AP);
      float Length_AB = AB.norm();
      float Length_ABCrossAP = ABcrossAP.norm();
      float H = Length_ABCrossAP / Length_AB;
      if( H < 0.1 ){
        pcl_source_ptr->erase(pcl_source_ptr_i);
        // ROS_INFO("Remove Cable Point Cloud H = %f", H);
      }
    }
  }

  // 処理後の点群をpublish
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = "world";
  fileterd_cloud_pub_.publish(filtered_pc2);

}

void CableRemove::DrawCable(Eigen::Vector3f cable_start_point, Eigen::Vector3f cable_end_point)
{

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/world";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "cable_line";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.02;


  line_list.color.r = 1.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 0.5;

  geometry_msgs::Point p;
  p.x = cable_start_point[0];
  p.y = cable_start_point[1];
  p.z = cable_start_point[2];

  line_list.points.push_back(p);

  p.x = cable_end_point[0];
  p.y = cable_end_point[1];
  p.z = cable_end_point[2];

  line_list.points.push_back(p);

  marker_pub_.publish(line_list);
}

void CableRemove::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
