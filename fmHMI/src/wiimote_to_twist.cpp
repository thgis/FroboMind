/*************************************************************************************
 # Copyright (c) 2011, Kent Stark Olsen
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 # 1. Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. All advertising materials mentioning features or use of this software
 #    must display the following acknowledgement:
 #    This product includes software developed by the University of Southern Denmark.
 # 4. Neither the name of the University of Southern Denmark nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY KENT STARK OLSEN ''AS IS'' AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL KENT STARK OLSEN BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************
 # File:    wiimote_to_twist.cpp
 # Author:  Kent Stark Olsen <kent.stark.olsen@gmail.com>
 # Created:     Jun 24, 2011 Kent Stark Olsen
 **************************************************************************************
 # Features:
 #  * Makes twist message of Wiimote accelerometer
 #  * Filters input from Wiimote (Runge Kutta 4th order)
 *************************************************************************************/

#include "ros/ros.h"
//#include "joy/Joy.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"
#include "boost/circular_buffer.hpp"

class WiiToTwist
{
public:
  //  Constructor
  WiiToTwist();

private:
  //  Node handler
  ros::NodeHandle global_n;
  ros::NodeHandle local_n;

  //      Subscribers/Publishers
  ros::Subscriber joy_sub;
  ros::Publisher twist_pub;

  //  Ring buffers for filters
  boost::circular_buffer<double> x_buffer;
  boost::circular_buffer<double> y_buffer;
  boost::circular_buffer<double> z_buffer;

  //  Twist message
  geometry_msgs::TwistStamped twist_msg;

  //  Parameters
  std::string subscriber_topic, publisher_topic;
  int normal_movement_button, slow_movement_button;
  int x_axis, y_axis, z_axis;
  bool invert_x_axis, invert_y_axis, invert_z_axis;
  double gravitation, max_linear_velocity, max_angular_velocity;
  double x_acceleration, y_acceleration, z_acceleration;
  double pitch, roll;
  double linear_velocity_x, angular_velocity_z;
  double scale_linear_velocity_x, scale_angular_velocity_z, scale_slow_velocity;

  //  Callback method
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

WiiToTwist::WiiToTwist()
{

  //      Instantiate nodehandlers
  global_n = ros::NodeHandle();
  local_n = ros::NodeHandle("~");

  //  Get max_linear_velocity from parameter server if it's avaiable, otherwise set to zero.
  if (local_n.hasParam("max_linear_velocity"))
  {
    local_n.getParam("max_linear_velocity", max_linear_velocity);
  }
  else
  {
    ROS_ERROR("Failed to get parameter 'max_linear_velocity', value will be set to zero.");
    max_linear_velocity = 0.0;
  }

  //  Get max_angular_velocity from parameter server if it's avaiable, otherwise set to zero.
  if (local_n.hasParam("max_angular_velocity"))
  {
    local_n.getParam("max_angular_velocity", max_angular_velocity);
  }
  else
  {
    ROS_ERROR("Failed to get parameter 'max_angular_velocity', value will be set to zero.");
    max_angular_velocity = 0.0;
  }

  //  Get parameters from parameter server if it's avaiable, otherwise set to default values.
  local_n.param("max_angular_velocity", max_angular_velocity, 0.0);
  local_n.param("max_linear_velocity", max_linear_velocity, 0.0);
  local_n.param("normal_movement_button", normal_movement_button, 3);
  local_n.param("slow_movement_button", slow_movement_button, 2);
  local_n.param("x_axis", x_axis, 0);
  local_n.param("y_axis", y_axis, 1);
  local_n.param("z_axis", z_axis, 2);
  local_n.param("invert_x_axis", invert_x_axis, false);
  local_n.param("invert_y_axis", invert_y_axis, false);
  local_n.param("invert_z_axis", invert_z_axis, false);
  local_n.param("scale_linear_velocity_x", scale_linear_velocity_x, 1.0);
  local_n.param("scale_angular_velocity_z", scale_angular_velocity_z, 1.0);
  local_n.param("scale_slow_velocity", scale_slow_velocity, 0.1);
  local_n.param<std::string> ("subscriber_topic", subscriber_topic, "joy");
  local_n.param<std::string> ("publisher_topic", publisher_topic, "wii_cmd_vel");

  x_buffer = boost::circular_buffer<double>(6);
  y_buffer = boost::circular_buffer<double>(6);
  z_buffer = boost::circular_buffer<double>(6);

  //  Subscriber and publisher
  joy_sub = global_n.subscribe<sensor_msgs::Joy> (subscriber_topic, 10, &WiiToTwist::joyCallback, this);
  twist_pub = global_n.advertise<geometry_msgs::TwistStamped> (publisher_topic, 1);
}

void WiiToTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //  Get values from wiimote axes
  x_buffer.push_back((double)joy -> axes[x_axis]);
  y_buffer.push_back((double)joy -> axes[y_axis]);
  z_buffer.push_back((double)joy -> axes[z_axis]);

  //  4th order Runge Kutta filter
  x_acceleration = (x_buffer[0] + 2 * x_buffer[1] + 2 * x_buffer[2] + x_buffer[3]) / 6.0;
  y_acceleration = (y_buffer[0] + 2 * y_buffer[1] + 2 * y_buffer[2] + y_buffer[3]) / 6.0;
  z_acceleration = (z_buffer[0] + 2 * z_buffer[1] + 2 * z_buffer[2] + z_buffer[3]) / 6.0;

  //  Invert values if required
  if (invert_x_axis)
    x_acceleration *= -1;
  if (invert_y_axis)
    y_acceleration *= -1;
  if (invert_z_axis)
    z_acceleration *= -1;

  //  Calculate pitch and roll
  pitch = atan2(x_acceleration, sqrt(pow(y_acceleration, 2) + pow(z_acceleration, 2))) / M_PI_2;
  roll = atan2(y_acceleration, sqrt(pow(x_acceleration, 2) + pow(z_acceleration, 2))) / M_PI_2;


  twist_msg.twist.linear.x = pitch * max_linear_velocity * scale_linear_velocity_x;

  twist_msg.twist.angular.z = roll * max_angular_velocity * scale_angular_velocity_z;

  twist_msg.twist.linear.y = 0.0;
  twist_msg.twist.linear.z = 0.0;

  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;

  // Time stamp message -andb
  twist_msg.header.stamp = ros::Time::now();

  // Publish twist message
  twist_pub.publish(twist_msg);
}

int main(int argc, char **argv)
{
  //  Initialize ROS
  ros::init(argc, argv, "wiimote_to_twist");

  //  Make instance of object
  WiiToTwist wii_to_twist = WiiToTwist();

  //  Loop
  ros::spin();
}
