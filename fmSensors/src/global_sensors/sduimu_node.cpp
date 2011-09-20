#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/serial.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"

#define DEG2RAD M_PI/180.0

sensor_msgs::Imu imu_msg;
geometry_msgs::Pose imu_pose_msg;
ros::Publisher imu_pub;
ros::Publisher imu_pose_pub;

  std::string frame_id;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

void imuCallback(const fmMsgs::serial::ConstPtr& msg)
{

  int count = 0;
  boost::char_separator<char> sep("$*,");
  tokenizer::iterator tok_iter; 
  tokenizer tokens(msg->data, sep);


  for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
 	count ++;

  if(count == 9){
  
   tok_iter = tokens.begin(); 
   *tok_iter++; //remove $RLIMU
   
   imu_msg.header = msg->header;
   imu_msg.header.frame_id = frame_id;

   imu_msg.linear_acceleration.x = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.linear_acceleration.y = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.linear_acceleration.z = boost::lexical_cast<int>(*tok_iter++) * 0.001 * 9.82;
   imu_msg.angular_velocity.x = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
   imu_msg.angular_velocity.y = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
   imu_msg.angular_velocity.z = boost::lexical_cast<int>(*tok_iter++) * 0.05 * DEG2RAD;
   
}else{
	ROS_WARN("ERROR");
  }


 // ROS_INFO("Got $RLIMU");

       imu_pub.publish(imu_msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_parser");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  std::string subscribe_topic_id;
  std::string publish_topic_id;

  n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "fmBSP/com1_msg");
  n.param<std::string> ("publish_topic_id", publish_topic_id, "imu_msg");
  n.param<std::string> ("frame_id", frame_id, "/base");

  ros::Subscriber sub = n.subscribe(subscribe_topic_id, 1, imuCallback);
  imu_pub = n.advertise<sensor_msgs::Imu> (publish_topic_id, 1);

  ros::spin();

  return 0;

}
