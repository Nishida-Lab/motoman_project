#include <euclidean_cluster.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "EuclideanClusterNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  EuclideanCluster cluster(nh, n);
  cluster.run();

  return 0;
}
