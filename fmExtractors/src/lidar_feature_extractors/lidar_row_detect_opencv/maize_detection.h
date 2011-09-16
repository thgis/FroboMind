/*
 * ochard_detection.h
 *
 *  Created on: Jun 5, 2011
 *      Author: soeni05
 */

#ifndef MAIZE_DETECTION_H_
#define MAIZE_DETECTION_H_


#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/Image.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/row.h"
#include "tf/transform_listener.h"

#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"


//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "visualization_msgs/Marker.h"

//#include <cv_bridge/CvBridge.h>


class MaizeDetector
{
private:



  IplImage* rawData_;
  IplImage* workingImg_;
  CvPoint point1, point2, point3, point4,point5;
  CvSeq* lines_;
  CvMemStorage* storage_;
  CvPoint intersection_;

  visualization_msgs::Marker marker_r;
  visualization_msgs::Marker marker_l;

  fmMsgs::row rows_;


  double    right_distance_;
  double    left_distance_;
  double    right_angle_;
  double	   left_angle_;
  double 	e_angle;
  double 	e_distance;

  double	row_dist_est;

  double 	right_angle_rolling_mean_[60];
  double 	right_dist_rolling_mean_[60];
  double 	left_angle_rolling_mean_[60];
  double 	left_dist_rolling_mean_[60];
  sensor_msgs::Image image_;

  void clearRawImage();
  int intersect(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4);

public:

  ros::Publisher pc_pub;
  ros::Publisher marker_r_pub;
  ros::Publisher marker_l_pub;

  ros::Publisher row_pub;

  ros::Subscriber laser_scan;

  laser_geometry::LaserProjection projector_;std::string image_topic_; //default output
  ros::Publisher image_pub_; //image message publisher
  MaizeDetector();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

};

#endif /* MAIZE_DETECTION_H_ */
