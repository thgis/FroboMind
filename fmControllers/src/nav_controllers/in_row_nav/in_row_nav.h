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
 # File: in_row_nav.h
 # Purpose: In-row navigation class
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#ifndef IN_ROW_NAV_H_
#define IN_ROW_NAV_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "fmMsgs/row.h"
#include "pid_regulator.h"

class IN_ROW_NAV {

private:

	PIDRegulator angle_regulator;
    PIDRegulator distance_regulator;

    geometry_msgs::TwistStamped twist_msg;

	double angle_regulator_output_;
	double distance_regulator_output_;

public:

	ros::Subscriber maize_row_sub_;
	ros::Publisher twist_pub_;

	std::string maize_sub_top_;
	std::string twist_pub_top_;

	void maizehandler(const fmMsgs::rowConstPtr & row_msg);

	IN_ROW_NAV();
	virtual ~IN_ROW_NAV();
};

#endif /* IN_ROW_NAV_H_ */
