/****************************************************************************
 # Behaviour selector node
 # Copyright (c) 2011 Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: behaviour_selector_node.cpp
 # Purpose: Determine best behaviour
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/
#include <ros/ros.h>
#include "behaviour_selector.h"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "behaviour_selector");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	//Create instance of object
	BEHAVIOUR_SELECTOR bs;

	//load parameters from parameterserver
	n.param<std::string> ("joy_topic", bs.joy_sub_top_, "/joy");
	n.param<std::string> ("autonomous_twist_topic", bs.twist_autonomous_sub_top_, "/auto_cmd_vel");
	n.param<std::string> ("manual_twist_topic", bs.twist_manual_sub_top_, "/wii_cmd_vel");
	n.param<std::string> ("test_twist_topic", bs.twist_test_sub_top_, "/test_cmd_vel");
	n.param<std::string> ("twist_topic", bs.twist_cmd_pub_top_, "/cmd_vel");

	//Subscribe to twist messages
	bs.twist_autonomous_subscriber_ = nh.subscribe<geometry_msgs::TwistStamped>(bs.twist_autonomous_sub_top_.c_str(), 1, &BEHAVIOUR_SELECTOR::twistautonomoushandler, &bs);
	bs.twist_manual_subscriber_ = nh.subscribe<geometry_msgs::TwistStamped>(bs.twist_manual_sub_top_.c_str(), 1, &BEHAVIOUR_SELECTOR::twistmanualhandler, &bs);
	bs.twist_test_subscriber_ = nh.subscribe<geometry_msgs::TwistStamped>(bs.twist_test_sub_top_.c_str(), 1, &BEHAVIOUR_SELECTOR::twisttesthandler, &bs);

	//Advertise cmd_vel twist message
	bs.twist_publisher_ = nh.advertise<geometry_msgs::TwistStamped>(bs.twist_cmd_pub_top_.c_str(),1);

	//Handle deadman button and joy watchdog funtionality
	bs.joy_subscriber_ = nh.subscribe<joy::Joy>(bs.joy_sub_top_, 1, &BEHAVIOUR_SELECTOR::modehandler, &bs);
	bs.wd_timer_ =  nh.createTimer(ros::Duration(0.01),&BEHAVIOUR_SELECTOR::modewd, &bs,0);

	//Handle callbacks
	ros::spin();



}

