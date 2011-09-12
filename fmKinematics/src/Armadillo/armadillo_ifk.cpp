#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include <boost/asio.hpp>
#include <sstream>
#include <string>
#include "joy/Joy.h" //dead man button
#include "fmMsgs/rtq.h"
#include "fmMsgs/rtq_command.h"
#include "geometry_msgs/TwistStamped.h"

// published data
fmMsgs::rtq_command rtq_command_msg_left;
fmMsgs::rtq_command rtq_command_msg_right;

//vic_msgs::Odometry //some kind of message type for odometry data

ros::Publisher hl_publisher;
ros::Subscriber hl_subscriber;
ros::Publisher ll_publisher_left;
ros::Publisher ll_publisher_right;
ros::Subscriber ll_subscriber_left;
ros::Subscriber ll_subscriber_right;


/*#define WII_BTN_A 2
#define WII_BTN_B 3

bool deadManActive = false;
bool slowMoveActive = true;
void callbackHandlerDeadManBtn(const joy::Joy::ConstPtr& joy)
{

  if (joy->buttons[WII_BTN_A]) // Slow move switch on WiiMote button A
    slowMoveActive = true;
  else slowMoveActive = false;

  if (joy->buttons[WII_BTN_B]) // Dead man switch on WiiMote button B
    deadManActive = true;
  else deadManActive = false;

}
*/

// Input is a twist message from highlevel
// Output is two rtq command messages, one for each wheel
void callbackHandlerHlSubscriber(const geometry_msgs::TwistStamped::ConstPtr& twi){

  //ROS_INFO("callbackHandlerHlSubscriber() [%lf,%lf,%lf]",twi->twist.linear.x,twi->twist.linear.y,twi->twist.linear.z);
  //ROS_INFO("callbackHandlerHlSubscriber() [%lf,%lf,%lf]",twi->twist.angular.x,twi->twist.angular.y,twi->twist.angular.z);



  // Forward Kinematics
  double W = 0.755; //length from center to meter
  double vel_right = twi->twist.linear.x - ( W * twi->twist.angular.z );
  double vel_left = - (twi->twist.linear.x + ( W * twi->twist.angular.z ));


  // Inverse Kinematics
  //double V = (vel_left + vel_right) / 2.0;
  //double omega = (vel_right/W) - (vel_left/W);

  rtq_command_msg_left.TrackSpeed = vel_left;
  rtq_command_msg_left.header.stamp = ros::Time::now();
  ll_publisher_left.publish(rtq_command_msg_left);

  rtq_command_msg_right.TrackSpeed = vel_right;
  rtq_command_msg_right.header.stamp = ros::Time::now();
  ll_publisher_right.publish(rtq_command_msg_right);

  //ROS_INFO("callbackHandlerHlSubscriber() [%lf,%lf]",vel_left,vel_right);

}



// Input is a rtq status message from lowlevel
// Output is odometry to highlevel
void callbackHandlerLlSubscriber(const fmMsgs::rtq::ConstPtr& twi){
  ROS_INFO("callbackHandlerLlSubscriber()");

}


int main(int argc, char **argv){

  ros::init(argc, argv, "armadillo_ifk");
  ros::NodeHandle nh("~"); //local nh

  std::string hl_publisher_topic;
  std::string hl_subscriber_topic;
  std::string ll_publisher_topic_left;
  std::string ll_publisher_topic_right;
  std::string ll_subscriber_topic_left;
  std::string ll_subscriber_topic_right;
  std::string deadmanbutton_topic;
  nh.param<std::string>("hl_publisher_topic", hl_publisher_topic, "hl_publisher_topic");
  nh.param<std::string>("hl_subscriber_topic", hl_subscriber_topic, "hl_subscriber_topic");
  nh.param<std::string>("ll_publisher_topic_left", ll_publisher_topic_left, "ll_publisher_topic_left");
  nh.param<std::string>("ll_publisher_topic_right", ll_publisher_topic_right, "ll_publisher_topic_right");
  nh.param<std::string>("ll_subscriber_topic_left", ll_subscriber_topic_left, "ll_subscriber_topic_left");
  nh.param<std::string>("ll_subscriber_topic_right", ll_subscriber_topic_right, "ll_subscriber_topic_right");
  nh.param<std::string>("deadmanbutton_topic", deadmanbutton_topic, "deadmanbutton_topic");

  //my_publisher = nh.advertise<vic_msgs::rtq> (publisher_topic.c_str(), 1); //seneste msg ligger klar i topic'en til fremtidige sucscribers
  hl_subscriber = nh.subscribe<geometry_msgs::TwistStamped> (hl_subscriber_topic.c_str(), 1, &callbackHandlerHlSubscriber); //seneste msg ligger klar i topic'en til fremtidige sucscribers
  //hl_publisher = nh.advertise<geometry_msgs::TwistStamped> (hl_publisher_topic.c_str(), 1);
  //ll_subscriber = nh.subscribe<vic_msgs::rtq> (ll_subscriber_topic.c_str(), 1, &callbackHandlerLlSubscriber);
  ll_publisher_left = nh.advertise<fmMsgs::rtq_command> (ll_publisher_topic_left.c_str(), 1);
  ll_publisher_right = nh.advertise<fmMsgs::rtq_command> (ll_publisher_topic_right.c_str(), 1);
  //deadmanbutton_subscriber = nh.subscribe<joy::Joy> (deadmanbutton_topic.c_str(), 1, &callbackHandlerDeadManBtn); //from rtq_testnode

  //callback_timer = nh.createTimer(ros::Duration(0.33), timerCallback);
  //wii_subscriber = nh.subscribe<> (subscriber_topic.c_str(), 1000, &serialInterface::writeHandler, &serialInterface);


  ROS_INFO("armadillo_ifk running");

  ros::spin(); //spinner på events der har noget at gøre med denne node (f.eks. timeren)

  return 0;
}
