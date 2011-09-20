#include <csignal>
#include <cstdio>

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "vic_msgs/can.h"
#include "vic_msgs/encoder.h"
#include "LindeLiendeEncoder.h"


/* published data */
vic_msgs::can can_tx_msg;
ros::Timer can_tx_timer;

LLENCODER llencoder;


void LLEncoderNodePeriodicCallback(const ros::TimerEvent& e)
{
  llencoder.watchdog();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "encoder_node");
  ros::NodeHandle nh("~");

  std::string enc_publisher_topic;
  std::string publisher_topic;
  std::string subscriber_topic;

  nh.param<std::string>("publisher_topic", publisher_topic, "/vic_interfaces/can0_tx");
  nh.param<std::string>("subscriber_topic", subscriber_topic, "/vic_interfaces/can0_rx");
  nh.param<std::string>("enc_publisher_topic", enc_publisher_topic, "/vic_sensors/encoder");

  nh.param<int>("encoder_id", llencoder.enc_id, 11);

  llencoder.can_tx_pub = nh.advertise<vic_msgs::can> (publisher_topic.c_str(), 1);
  llencoder.enc_ticks_pub = nh.advertise<vic_msgs::encoder> (enc_publisher_topic.c_str(), 1);
  llencoder.can_rx_sub = nh.subscribe<vic_msgs::can> (subscriber_topic.c_str(), 1000, &LLENCODER::processCanRxEvent, &llencoder);

  llencoder.processCanTxEvent(); // to kickstart the initialization;

  can_tx_timer = nh.createTimer(ros::Duration(0.1), LLEncoderNodePeriodicCallback);



  ros::spin();

  return 0;
}


