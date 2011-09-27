#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/gpgga.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"

#define DEG2RAD M_PI/180.0

ros::Publisher gpgga_pub;
std::string frame_id;

fmMsgs::gpgga gpgga_msg;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

void gpsCallback(const fmMsgs::serial::ConstPtr& msg)
{

  int count = 0;
  boost::char_separator<char> sep("*,");
  tokenizer::iterator tok_iter; 
  tokenizer tokens(msg->data, sep);


  for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
 	count ++;

   tok_iter = tokens.begin(); 
  
  if (*tok_iter == "$GPGGA")
  {
	*tok_iter++;
	gpgga_msg.header.stamp = ros::Time::now();
	gpgga_msg.time = *tok_iter;
	*tok_iter++;
	
	gpgga_pub.publish(gpgga_msg);


/*
   *tok_iter++; //remove $RLIMU
   
   imu_msg.header = msg->header;
   imu_msg.header.frame_id = frame_id;

   imu_msg.linear_acceleration.x = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.linear_acceleration.y = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.linear_acceleration.z = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.angular_velocity.x = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
   imu_msg.angular_velocity.y = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
   imu_msg.angular_velocity.z = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
 // ROS_INFO("Got $RLIMU");

       imu_pub.publish(imu_msg);

*/   
  }



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_parser");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  std::string subscribe_topic_id;
  std::string publish_topic_id;

  n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "fmBSP/gps_msg");
  n.param<std::string> ("publish_topic_id", publish_topic_id, "gpgga_msg");
  n.param<std::string> ("frame_id", frame_id, "/base");

  ros::Subscriber sub = n.subscribe(subscribe_topic_id, 1, gpsCallback);
  gpgga_pub = n.advertise<fmMsgs::gpgga> (publish_topic_id, 1);

  ros::spin();

  return 0;

}

