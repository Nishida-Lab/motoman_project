#include <kinect_downsampler.hpp>

KinectV2Downsampler::KinectV2Downsampler(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh), rate_(n.param("loop_rate", 10)),
    frame_id_(n.param<std::string>("resized_pc_frame_id", "/resized_pc_frame"))
{
  leaf_size_x_ = n.param("leaf_size_x", 0.3);
  leaf_size_y_ = n.param("leaf_size_y", 0.3);
  leaf_size_z_ = n.param("leaf_size_z", 0.3);
  
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/source_pointcloud"), 1, &KinectV2Downsampler::resizeCallback, this);
  resized_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("resized_pc_topic_name", "/resized_pointcloud"), 1);
}

void KinectV2Downsampler::resizeCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_source;
  pcl::fromROSMsg(*source_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr resized_pc_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (leaf_size_x_, leaf_size_y_, leaf_size_z_);
  approximate_voxel_filter.setInputCloud (pcl_source_ptr);
  approximate_voxel_filter.filter (*resized_pc_ptr);

  sensor_msgs::PointCloud2 resized_pc;
  pcl::toROSMsg(*resized_pc_ptr, resized_pc);
  resized_pc.header.stamp = ros::Time::now();
  resized_pc.header.frame_id = frame_id_;
  resized_pc_pub_.publish(resized_pc);
}

void KinectV2Downsampler::run()
{
  while(nh_.ok())
    {
      ros::spinOnce();
      rate_.sleep();
    }
}
