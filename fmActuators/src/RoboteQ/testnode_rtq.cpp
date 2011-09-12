#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include <boost/asio.hpp>
#include <sstream>
#include <string>
#include "fmMsgs/rtq.h"
#include "geometry_msgs/TwistStamped.h"

//TODO: add wiimote joy as input


/* published data */
//vic_msgs::rtq rtq_msg; 						//old RoboTeq rtq node message
geometry_msgs::TwistStamped rtq_twi_msg;	//new Twist based rtw node message

ros::Publisher my_publisher;
ros::Timer callback_timer;

/*void timerCallback(const ros::TimerEvent& e) // old
{

  rtq_msg.sRPM = 555;
  rtq_msg.header.stamp = ros::Time::now();
  my_publisher.publish(rtq_msg);

}*/

double xprev = 0.0;
double xdelta = 5.0;

void timerCallback(const ros::TimerEvent& e) // new Twist based
{

  //if ( (xprev > 990.0) || (xprev < -990.0) ) xdelta = -xdelta;
  if ( (xprev > 150.0) || (xprev < -150.0) ) xdelta = -xdelta;

  xprev += xdelta;

  rtq_twi_msg.twist.linear.x = xprev;
  rtq_twi_msg.header.stamp = ros::Time::now();
  my_publisher.publish(rtq_twi_msg);

  //ROS_INFO("Yo");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "testnode_rtq");
  ros::NodeHandle nh("~"); //local nh

  std::string publisher_topic;

  nh.param<std::string>("publisher_topic", publisher_topic, "rtq_msg"); //topic'en hedder default "rtq_msg"

  //my_publisher = nh.advertise<vic_msgs::rtq> (publisher_topic.c_str(), 1); //seneste msg ligger klar i topic'en til fremtidige sucscribers
  my_publisher = nh.advertise<geometry_msgs::TwistStamped> (publisher_topic.c_str(), 1); //seneste msg ligger klar i topic'en til fremtidige sucscribers
  callback_timer = nh.createTimer(ros::Duration(0.33), timerCallback);

  //wii_subscriber = nh.subscribe<> (subscriber_topic.c_str(), 1000, &serialInterface::writeHandler, &serialInterface);


  ROS_INFO("testnode_rtq running");

  ros::spin(); //spinner på events der har noget at gøre med denne node (f.eks. timeren)

  return 0;
}

