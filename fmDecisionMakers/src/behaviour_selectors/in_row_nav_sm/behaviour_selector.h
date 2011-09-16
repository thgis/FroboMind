/****************************************************************************
# Behaviour selector class header
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
# File: behaviour_selector.h
# Purpose: Determine best behaviour
# Project: Field Robot - Vehicle Interface Computer
# Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
# Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
****************************************************************************/

#ifndef BEHAVIOUR_SELECTOR_H_
#define BEHAVIOUR_SELECTOR_H_

#include <joy/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#define WII_AXIS_Z 2
#define WII_BTN_A 2
#define WII_BTN_B 3
#define WII_BTN_MINUS 5
#define WII_BTN_HOME 10
#define WII_BTN_PLUS 4

#define WD_MAX_VALUE 25

enum
{
  BS_DEADMAN_DEACTIVATED_MODE = 0, BS_MANUAL_MODE, BS_AUTONOMOUS_MODE, BS_TEST_MODE
};


class BEHAVIOUR_SELECTOR {

private:

	bool wii_btn_plus_pushed ;
	bool wii_btn_minus_pushed ;

	int mode_wd_ ;
	int bs_state_;
	int bs_old_state_;

	geometry_msgs::TwistStampedConstPtr cmd_vel_msg;
	geometry_msgs::TwistStamped wd_vel_msg;


public:

	ros::Subscriber joy_subscriber_;
	ros::Subscriber twist_autonomous_subscriber_;
	ros::Subscriber twist_manual_subscriber_;
	ros::Subscriber twist_test_subscriber_;

	ros::Publisher twist_publisher_;

	std::string twist_autonomous_sub_top_;
	std::string twist_manual_sub_top_;
	std::string twist_test_sub_top_;
	std::string twist_cmd_pub_top_;

	std::string joy_sub_top_;

	std::string twist_pub_top_;

	ros::Timer wd_timer_;

	BEHAVIOUR_SELECTOR();
	~BEHAVIOUR_SELECTOR();

	void modewd(const ros::TimerEvent& e);
	void modehandler(const joy::JoyConstPtr & joy_msg);
	void twistautonomoushandler(const geometry_msgs::TwistStampedConstPtr & twist_msg);
	void twistmanualhandler(const geometry_msgs::TwistStampedConstPtr & twist_msg);
	void twisttesthandler(const geometry_msgs::TwistStampedConstPtr & twist_msg);


};

#endif /* BEHAVIOUR_SELECTOR_H_ */
