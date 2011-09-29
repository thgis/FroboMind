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
 # File:    pid_regulator.cpp
 # Author:  Kent Stark Olsen <kent.stark.olsen@gmail.com>
 # Created: Jun 28, 2011 Kent Stark Olsen
 **************************************************************************************
 # Features:
 #  * PID Regulator Class
 *************************************************************************************/
#include "pid_regulator.h"

PIDRegulator::PIDRegulator()
{
	//	Setup coefficients (set all to zero)
	p_coef = 0;
	i_coef = 0;
	d_coef = 0;

	//	Setup ringbuffers
	time_now = boost::circular_buffer<double>(2);
	errors = boost::circular_buffer<double>(2);
}

PIDRegulator::PIDRegulator(double p, double i, double d)
{
	//	Setup coefficients
	p_coef = p;
	i_coef = i;
	d_coef = d;

	//	Setup ringbuffers
	time_now = boost::circular_buffer<double>(2);
	errors = boost::circular_buffer<double>(2);

	time_now.push_back(0);
	time_now.push_back(0);
	errors.push_back(0);
	errors.push_back(0);
}

double PIDRegulator::update(double feedback, double setpoint)
{

	ROS_INFO("update called");

	//	Put error and time in ring buffer
	time_now.push_back(ros::Time::now().toSec());
	ROS_INFO("time pushed back %f" ,time_now[0] );
	errors.push_back(setpoint - feedback);
	ROS_INFO("error %f" ,errors[0] );
	dt = (time_now[1]-time_now[0]);
	ROS_INFO("dt calculated %f", dt);

	//	Calculate P-term
	p_term = p_coef * errors[0];

	//	Calculate I-term
	i_term += i_coef * errors[0] * dt;

	//	Calculate D-term
	d_term = d_coef * (errors[0] - errors[1]) * dt;

	//	Return PID
	return p_term + i_term + d_term;
}

double PIDRegulator::getP ()
{
	return p_coef;
}
void PIDRegulator::setP (double p)
{
	p_coef = p;
}

double PIDRegulator::getI ()
{
	return i_coef;
}
void PIDRegulator::setI (double i)
{
	i_coef = i;
}

double PIDRegulator::getD ()
{
	return d_coef;
}
void PIDRegulator::setD (double d)
{
	d_coef = d;
}
