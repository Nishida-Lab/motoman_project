#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PointCloudMerger {
public:
    PointCloudMerger(ros::NodeHandle &nh, ros::NodeHandle &privNh);
    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg1, const sensor_msgs::PointCloud2::ConstPtr &msg2);

protected:

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloudSubscriber1;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloudSubscriber2;

    message_filters::Synchronizer<MySyncPolicy> sync;

    ros::Publisher  mergedPointcloudPublisher;
};

PointCloudMerger::PointCloudMerger(ros::NodeHandle &nh, ros::NodeHandle &privNh) :
    pointcloudSubscriber1(nh,
                         privNh.param<std::string>("/cloud_in_1", "/kinect_first/kinect2/hd/points"),
                         1),
    pointcloudSubscriber2(nh,
                         privNh.param<std::string>("/cloud_in_2", "/kinect_second/kinect2/hd/points"),
                         1),
    mergedPointcloudPublisher(nh.advertise<sensor_msgs::PointCloud2>(
                         privNh.param<std::string>("/cloud_out", "/merged_cloud"),
                         1)),
    sync(MySyncPolicy(10), pointcloudSubscriber1, pointcloudSubscriber2)
{
    sync.registerCallback(boost::bind (&PointCloudMerger::callback, this, _1, _2));
}

void PointCloudMerger::callback(const sensor_msgs::PointCloud2::ConstPtr& msg1, const sensor_msgs::PointCloud2::ConstPtr& msg2) {
    static tf::TransformListener listener;

    sensor_msgs::PointCloud2 cloud_transformed;
    listener.waitForTransform(msg2->header.frame_id, msg1->header.frame_id, msg1->header.stamp, ros::Duration(1.0));
    pcl_ros::transformPointCloud(msg2->header.frame_id, *msg1, cloud_transformed, listener);

    sensor_msgs::PointCloud2 cloud_merged;
    pcl::concatenatePointCloud(cloud_transformed, *msg2, cloud_merged);
    mergedPointcloudPublisher.publish(cloud_merged);
    ROS_INFO("Point cloud published");
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger");
    ros::NodeHandle nh;
    ros::NodeHandle privNh("~");

    PointCloudMerger pcm(nh, privNh);
    (void) pcm;

    ros::spin();

    return 0;
}

