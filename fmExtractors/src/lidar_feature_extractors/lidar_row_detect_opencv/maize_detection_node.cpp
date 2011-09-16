/****************************************************************************
# Lidar row detect node
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
# File: AES25_node.cpp
# Purpose: AES25 actuator driver.
# Project: Field Robot - Vehicle Interface Computer
# Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
# Created: Jun 3, 2011 Søren Hundevadt Nielsen, Source written
****************************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


#include "maize_detection.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "orchard_detector");

  MaizeDetector md;

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  std::string subscriber_topic;

  n.param<std::string>("subscriber_topic", subscriber_topic, "/fmSensors/laser_msgs_front");
  n.param<std::string>("image_topic_", md.image_topic_, "/image");


  md.marker_r_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_r", 1);
  md.marker_l_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_l", 1);

  md.row_pub = nh.advertise<fmMsgs::row>("/rows",1);
  //md.row_right_pub = nh.advertise<fmMsgs::row>("/right_row",1);

  md.laser_scan = nh.subscribe<sensor_msgs::LaserScan> (subscriber_topic.c_str(), 2, &MaizeDetector::processLaserScan, &md);
  md.pc_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 50);
  md.image_pub_ = nh.advertise<sensor_msgs::Image> (md.image_topic_, 30);



  //toPose.encmder_left_sub = nh.subscribe<fmMsgs::encmder> (enc_l_subscriber_topic.c_str(), 1000, &ToPose::processEncmderLeft, &toPose);
  //can_tx_timer = nh.createTimer(ros::Duration(0.1), LLEncmderNmdePeriicCallback);

  ros::spin();

  return 0;
}
