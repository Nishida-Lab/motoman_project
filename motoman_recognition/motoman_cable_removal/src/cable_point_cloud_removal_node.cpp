#include <cable_point_cloud_removal.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "CableRemoveNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  CableRemove cable(nh, n);
  cable.run();

  return 0;
}
