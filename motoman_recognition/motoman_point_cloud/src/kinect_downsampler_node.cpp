#include <kinect_downsampler.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Kinect Dowmsampler Node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  KinectV2Downsampler sampler(nh, n);
  sampler.run();
  
  return 0;
}
