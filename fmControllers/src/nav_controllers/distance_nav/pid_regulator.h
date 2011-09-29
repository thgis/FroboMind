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
 # File:    pid_regulator.h
 # Author:  Kent Stark Olsen <kent.stark.olsen@gmail.com>
 # Created: Jun 28, 2011 Kent Stark Olsen
 **************************************************************************************
 # Features:
 #  * PID Regulator Class
 *************************************************************************************/
#include "ros/ros.h"
#include "boost/circular_buffer.hpp"

class PIDRegulator
{
public:
	//	Constructor
	PIDRegulator ();
	PIDRegulator (double p, double i, double d);

	//	Methods
	double	update 	(double feedback, double setpoint);

	double 	getP ();
	void 	setP (double p);
	double 	getI ();
	void 	setI (double p);
	double 	getD ();
	void 	setD (double p);

private:
	//	Coefficients for regulation and terms
	double p_coef, i_coef, d_coef;
	double p_term, i_term, d_term;

	//	Delta time
	double dt;

	//	Ringbuffers for time and error
	boost::circular_buffer<double> 	time_now;
	boost::circular_buffer<double> 		errors;
};
