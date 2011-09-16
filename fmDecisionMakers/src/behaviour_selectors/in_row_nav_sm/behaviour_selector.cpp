/****************************************************************************
 # Behaviour selector class
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
 # File: behaviour_selector.cpp
 # Purpose: Determine best behaviour
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#include "behaviour_selector.h"

BEHAVIOUR_SELECTOR::BEHAVIOUR_SELECTOR() {

	bs_state_ = BS_DEADMAN_DEACTIVATED_MODE;
	bs_old_state_  = BS_DEADMAN_DEACTIVATED_MODE;
	mode_wd_ = 0;
}

BEHAVIOUR_SELECTOR::~BEHAVIOUR_SELECTOR() {

}

void BEHAVIOUR_SELECTOR::twistautonomoushandler(const geometry_msgs::TwistStampedConstPtr & twist_msg){
	if(bs_state_ == BS_AUTONOMOUS_MODE){
		//cmd_vel_msg = twist_msg;
		//twist_publisher_.publish(*cmd_vel_msg);
	}
}
void BEHAVIOUR_SELECTOR::twistmanualhandler(const geometry_msgs::TwistStampedConstPtr & twist_msg){
	if(bs_state_ == BS_MANUAL_MODE){
		cmd_vel_msg = twist_msg;
		twist_publisher_.publish(*cmd_vel_msg);
	}
}
void BEHAVIOUR_SELECTOR::twisttesthandler(const geometry_msgs::TwistStampedConstPtr & twist_msg){
	if(bs_state_ == BS_TEST_MODE){
		cmd_vel_msg = twist_msg;
		twist_publisher_.publish(*cmd_vel_msg);
	}
}

void BEHAVIOUR_SELECTOR::modehandler(const joy::JoyConstPtr & joy_msg) {

	if (joy_msg->buttons[WII_BTN_PLUS] && wii_btn_plus_pushed == false) {

		wii_btn_plus_pushed = true;

		switch (bs_state_) {
		case BS_DEADMAN_DEACTIVATED_MODE:
			bs_state_ = BS_MANUAL_MODE;
			break;
		case BS_MANUAL_MODE:
			bs_state_ = BS_AUTONOMOUS_MODE;
			break;
		case BS_AUTONOMOUS_MODE:
			bs_state_ = BS_TEST_MODE;
			break;
		case BS_TEST_MODE:
			bs_state_ = BS_MANUAL_MODE;
			break;
		}
	} else if (!joy_msg->buttons[WII_BTN_PLUS]) {
		wii_btn_plus_pushed = false;
	}

	if (joy_msg->buttons[WII_BTN_MINUS] && wii_btn_minus_pushed == false) {

		wii_btn_minus_pushed = true;

		switch (bs_state_) {
		case BS_DEADMAN_DEACTIVATED_MODE:
			bs_state_ = BS_TEST_MODE;
			break;
		case BS_MANUAL_MODE:
			bs_state_ = BS_TEST_MODE;
			break;
		case BS_AUTONOMOUS_MODE:
			bs_state_ = BS_MANUAL_MODE;
			break;
		case BS_TEST_MODE:
			bs_state_ = BS_AUTONOMOUS_MODE;
			break;
		}
	} else if (!joy_msg->buttons[WII_BTN_MINUS]) {
		wii_btn_minus_pushed = false;
	}

	if (!joy_msg->buttons[WII_BTN_B]) {
        if(!bs_state_ == BS_DEADMAN_DEACTIVATED_MODE){
        	bs_old_state_ = bs_state_;
        }
		bs_state_ = BS_DEADMAN_DEACTIVATED_MODE;
	}else{
		if(bs_state_ == BS_DEADMAN_DEACTIVATED_MODE){
			bs_state_ = bs_old_state_;
		}

	}

	mode_wd_ = WD_MAX_VALUE;
}

void BEHAVIOUR_SELECTOR::modewd(const ros::TimerEvent& e) {

	if(bs_state_ == BS_DEADMAN_DEACTIVATED_MODE){
		wd_vel_msg.twist.angular.z = 0;
		wd_vel_msg.twist.linear.x = 0;
		twist_publisher_.publish(wd_vel_msg);
	}

	if (mode_wd_ > 0) {

		if (--mode_wd_ == 0) {
			bs_state_ = BS_DEADMAN_DEACTIVATED_MODE;
			ROS_ERROR("LOST COMMUNICATION WITH WIIMOTE - FIX IT!");
		}
	}

	ROS_INFO("MODE: %d , watchdog counter %d",bs_state_,mode_wd_);
}
