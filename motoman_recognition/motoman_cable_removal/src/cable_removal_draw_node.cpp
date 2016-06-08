#include <cable_removal_draw.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "CableDrawNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  CableDraw draw(nh, n);
  draw.run();

  return 0;
}
