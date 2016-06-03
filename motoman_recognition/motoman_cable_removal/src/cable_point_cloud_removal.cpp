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
  bounding_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(n.param<std::string>("box_name", "/boundingbox_result"), 1);

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

  // std::cout << "(x, y, z) = ("
  //           << transform.getOrigin().x()
  //           << ", "
  //           << transform.getOrigin().y()
  //           << ", "
  //           << transform.getOrigin().z()
  //           << ")"
  //           << std::endl;

  Eigen::Vector3d dhand_adapter_pos;
  dhand_adapter_pos[0] = transform.getOrigin().x();
  dhand_adapter_pos[1] = transform.getOrigin().y();
  dhand_adapter_pos[2] = transform.getOrigin().z();
  Eigen::Vector3d A(0.36, 0, 1.56);
  sensor_msgs::PointCloud2 trans_pc;

  DrawCable(A, dhand_adapter_pos);
  double length = (dhand_adapter_pos - A).norm();
  Eigen::Affine3f pose = DrawBox(A, dhand_adapter_pos, length);

  try {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));

  // for(pcl::PointCloud<PointXYZ>::iterator pcl_source_ptr_i = pcl_source_ptr->begin(); pcl_source_ptr_i < pcl_source_ptr->end(); ++pcl_source_ptr_i){
  //   if (pcl_source_ptr_i->z > dhand_adapter_pos[2]){
  //     Eigen::Vector3f AB(dhand_adapter_pos[0]-A[0], dhand_adapter_pos[1]-A[1], dhand_adapter_pos[2]-A[2]);
  //     Eigen::Vector3f AP(pcl_source_ptr_i->x-A[0], pcl_source_ptr_i->y-A[1], pcl_source_ptr_i->z-A[2]);
  //     Eigen::Vector3f ABcrossAP = AB.cross(AP);
  //     float Length_AB = AB.norm();
  //     float Length_ABCrossAP = ABcrossAP.norm();
  //     float H = Length_ABCrossAP / Length_AB;
  //     if( H < 1 ){
  //       pcl_source_ptr->erase(pcl_source_ptr_i);
  //       // ROS_INFO("Remove Cable Point Cloud H = %f", H);
  //     }
  //   }
  //   pcl_source_ptr->erase(pcl_source_ptr_i);
  // }
  CropBox(pcl_source_ptr, pose, length);


  // 処理後の点群をpublish
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = "world";
  fileterd_cloud_pub_.publish(filtered_pc2);

}

void CableRemove::DrawCable(Eigen::Vector3d a, Eigen::Vector3d b)
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

  line_list.color.r = 1.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 0.75;

  geometry_msgs::Point p;
  p.x = a[0];
  p.y = a[1];
  p.z = a[2];

  line_list.points.push_back(p);

  p.x = b[0];
  p.y = b[1];
  p.z = b[2];

  line_list.points.push_back(p);

  marker_pub_.publish(line_list);
}

Eigen::Affine3f CableRemove::DrawBox(Eigen::Vector3d a, Eigen::Vector3d b, double length)
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
  Eigen::Quaternionf qf = q.cast <float> ();
  // スケーリング
  Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(1.0f, 1.0f, 1.0f);
  // 平行移動(x, y, z)
  Eigen::Translation<float, 3> translation = Eigen::Translation<float, 3>(float(a[0] + b[0])/2.0, float(a[1] + b[1])/2.0, float(a[2] + b[2])/2.0);
  // アフィン変換用行列
  Eigen::Affine3f matrix;
  // 変換行列を求める
  matrix = translation * scaling * qf;

  geometry_msgs::Pose geometry_pose;
  geometry_msgs::Vector3 size;

  geometry_pose.position.x = (a[0] + b[0])/2.0;
  geometry_pose.position.y = (a[1] + b[1])/2.0;
  geometry_pose.position.z = (a[2] + b[2])/2.0;
  geometry_pose.orientation.x = q.x();
  geometry_pose.orientation.y = q.y();
  geometry_pose.orientation.z = q.z();
  geometry_pose.orientation.w = q.w();

  size.x = 0.1;
  size.y = 0.1;
  size.z = length;

  jsk_recognition_msgs::BoundingBox box;
  box.header.frame_id = frame_id_;
  box.pose = geometry_pose;
  box.dimensions = size;

  jsk_recognition_msgs::BoundingBoxArray box_array;
  box_array.boxes.push_back(box);
  box_array.header.stamp = ros::Time::now();
  box_array.header.frame_id = frame_id_;

  bounding_pub_.publish(box_array);
  return matrix;
}

void CableRemove::CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Affine3f matrix, double length) {

  Eigen::Vector4f minPoint;
  minPoint[0] = -1; // define minimum point x
  minPoint[1] = -1; // define minimum point y
  minPoint[2] = 0; // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = 1; // define max point x
  maxPoint[1] = 1; // define max point y
  maxPoint[2] = length; // define max point z

  pcl::CropBox<pcl::PointXYZ> cropFilter(true);
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTransform(matrix);

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
