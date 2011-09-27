/*************************************************************************************
# Copyright (c) 2011, Søren Hundevadt Nielsen
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this software
# must display the following acknowledgement:
# This product includes software developed by the University of Southern Denmark.
# 4. Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIESLSEN ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL SØREN HUNDEVADT NIELSEN BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************
# File: isb_ts_node.cpp
# Purpose: isb time server node
# Project: vic_base
# Author: Søren Hundevadt Nielsen <soeni05@gmail.com>
# Created: Apr 29, 2011 Søren Hundevadt Nielsen, Source written
*************************************************************************************/
#include "ros/ros.h"
#include "fmMsgs/can.h"
#include "isb_pseudo_ts.h"


ros::Timer tsTimer;
ISBTS isb_ts;


void isbTSCallback(const ros::TimerEvent& e)
{
	isb_ts.tsUpdate();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "isb_ts_node");
  ros::NodeHandle nh("~");

  std::string publisher_topic;
  std::string subscriber_topic;
  std::string acc_publisher_topic;


  nh.param<std::string>("publisher_topic", publisher_topic, "/can_tx");
  nh.param<bool>("logstate", isb_ts.logstate_, false);

  isb_ts.can_tx_pub = nh.advertise<fmMsgs::can> (publisher_topic.c_str(), 1);

  tsTimer = nh.createTimer(ros::Duration(0.01), isbTSCallback);

  ros::spin();

  return 0;
}


