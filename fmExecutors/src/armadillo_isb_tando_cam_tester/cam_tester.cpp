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
 # File: cam_tester.cpp
 # Purpose: Camera tester class
 # Project: Field Robot - FroboMind Executors
 # Author: Anders Bøgild <andb@mmmi.sdu.dk>
 # Created: Sep 29, 2011 Anders Bøgild, Source copied and adapted from in_row_nav
 ****************************************************************************/

#include "cam_tester.hpp"

CamTestExecutor::CamTestExecutor(){

	Client client("move_distance",true); //true -> dont need ros::spin()

	client.waitForServer();
	fmControllers::MoveDistGoal goal;

	goal.move_distance_meter = 0.5;

	client.sendGoal(goal);
	client.waitForResult(ros::Duration(5.0));
	if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Goal succeeded");


}

CamTestExecutor::~CamTestExecutor(){

}

void CamTestExecutor::sendGoal(float meter){

	ROS_INFO("Set MoveDistAction goal");



}

