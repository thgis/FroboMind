/****************************************************************************
 # In row navigation
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
 # File: distance_nav.hpp
 # Purpose: Distance navigation class
 # Project: Field Robot - FroboMind controllers
 # Author: Anders Bøgild <andb@mmmi.sdu.dk>
 # Created: Sep 29, 2011 Anders Bøgild, Source copied and adapted from in_row_nav
 ****************************************************************************/

#ifndef DISTANCE_NAV_H_
#define DISTANCE_NAV_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "fmMsgs/row.h"
#include "pid_regulator.h"

#include <actionlib/server/simple_action_server.h>
#include <fmControllers/MoveDistAction.h>

typedef actionlib::SimpleActionServer<fmControllers::MoveDistAction> Server;


class DistanceNavigator {

private:

	//PIDRegulator angle_regulator;
    //PIDRegulator distance_regulator;

    geometry_msgs::TwistStamped twist_msg;

	double angle_regulator_output_;
	double distance_regulator_output_;

public:

	ros::Subscriber dist_sub_;
	ros::Publisher twist_pub_;

	std::string dist_sub_top_;
	std::string twist_pub_top_;

	Server server;

	void executeMoveDist(const fmControllers::MoveDistActionConstPtr& goal, Server* as);

	void distanceHandler(const fmMsgs::rowConstPtr & dist_msg);

	DistanceNavigator();
	virtual ~DistanceNavigator();
};

#endif /* DISTANCE_NAV_H_ */
