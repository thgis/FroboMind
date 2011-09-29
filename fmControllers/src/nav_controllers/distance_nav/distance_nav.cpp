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
 # File: distance_nav.cpp
 # Purpose: Distance navigation node
 # Project: Field Robot - FroboMind controllers
 # Author: Anders Bøgild <andb@mmmi.sdu.dk>
 # Created: Sep 29, 2011 Anders Bøgild, Source copied and adapted from in_row_nav
 ****************************************************************************/

#include "distance_nav.hpp"

DistanceNavigator::DistanceNavigator(){
	//angle_regulator = PIDRegulator(2.5,0,0);
	//distance_regulator = PIDRegulator(0.2,0,0);



}

DistanceNavigator::~DistanceNavigator(){

}

void DistanceNavigator::executeMoveDist(const fmControllers::MoveDistActionConstPtr& goal, Server* as){

	ROS_INFO("Got MoveDistAction");

	//Make the requested action happend

	as->setSucceeded();

}

void DistanceNavigator::distanceHandler(const fmMsgs::rowConstPtr & dist_msg){

	//distance_regulator_output_ = distance_regulator.update(maize_msg->error_distance,0);
	//angle_regulator_output_ = angle_regulator.update(maize_msg->error_angle,0);

	twist_msg.header.stamp = ros::Time::now();
	twist_msg.twist.linear.x=0.5;
	twist_msg.twist.angular.z=0; // for now we just drive in a straigt line... good luck!


	twist_pub_.publish(twist_msg);


	//ROS_INFO("dist from gps: %f" ,dist_msg->dist);




}
