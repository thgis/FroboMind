/****************************************************************************
# In row navigation node
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
# File: distance_nav_node.cpp
# Purpose: Distance navigation node
# Project: Field Robot - FroboMind controllers
# Author: Anders Bøgild <andb@mmmi.sdu.dk>
# Created: Sep 29, 2011 Anders Bøgild, Source copied and adapted from in_row_nav_node
****************************************************************************/
#include "distance_nav.hpp"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "distance_navigation");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	DistanceNavigator dn;

	n.param<std::string> ("maize_sub", dn.dist_sub_top_, "");
	n.param<std::string> ("twist_top", dn.twist_pub_top_, "/auto_cmd_vel");

	dn.twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(dn.twist_pub_top_.c_str(),1);
	//irn.maize_row_sub_ = nh.subscribe<fmMsgs::row>(irn.maize_sub_top_.c_str(),100,&IN_ROW_NAV::maizehandler, &irn);

	//Handle callbacks
	ros::spin();

}
