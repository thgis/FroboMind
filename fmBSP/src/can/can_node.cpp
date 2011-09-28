#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "vicCan.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_node");

  VicCan vicCan;
  if (vicCan.init("/dev/can0"))
    ros::spin();
  return 0;
}
