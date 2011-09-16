/****************************************************************************
 # AES25 node
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
 # Created: Jun 5, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#include "maize_detection.h"

MaizeDetector::MaizeDetector() {
	rawData_ = cvCreateImage(cvSize(600, 600), 8, 3);
	workingImg_ = cvCreateImage(cvSize(600, 600), 8, 1);

	row_dist_est = 0.75;
	for(int i=0;i<60;i++){
		right_angle_rolling_mean_[i] = 0;
		  right_dist_rolling_mean_[i] = 0;
		  left_angle_rolling_mean_[i] = 0;
		  left_dist_rolling_mean_[i] = 0;
	}
}

void MaizeDetector::clearRawImage() {
	cvZero(rawData_);
	storage_ = cvCreateMemStorage(0);
	lines_ = 0;

}

void MaizeDetector::processLaserScan(



	const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
	float rthetamean = 0, rrhomean = 0, lthetamean = 0, lrhomean = 0, theta = 0, rho = 0;
	double x0 = 0, y0 = 0, a, b;
	int lmc = 0, rmc = 0;

	static int right_count = 0;
	static int left_count = 0;

	clearRawImage();

	tf::TransformListener listener_;
	sensor_msgs::PointCloud cloud;


    try
    {
    	projector_.projectLaser(*laser_scan, cloud);
       // projector_.transformLaserScanToPointCloud("base_link",*laser_scan,cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }


	//ROS_INFO("Got it right");
	int size = cloud.points.size();

	for (int i = 0; i < size; i++) {
		//if (((abs(cloud.points[i].y))-((cloud.points[i].x-300)*0.5))<0.65) {
		if (abs((cloud.points[i].y))<0.65 && cloud.points[i].x >0 ) {


			point1.x = ((int)(cloud.points[i].x * 50) + 300);
			point1.y = ((int)(cloud.points[i].y * 50) + 300);
			point2.x = point1.x-4;
			point2.y = point1.y;
			cvLine(rawData_, point1, point2, CV_RGB(255,255,255), 3, CV_AA, 0);

		}
	}

	cvCvtColor(rawData_, workingImg_, CV_BGR2GRAY);
	cvDilate(rawData_,rawData_,NULL,6);
	cvErode(rawData_,rawData_,NULL,4);


	lines_ = cvHoughLines2(workingImg_, storage_, CV_HOUGH_STANDARD, 1, CV_PI/ 180, 60, 0, 0);
	//lines_ = cvHoughLines2(workingImg_, storage_, CV_HOUGH_PROBABILISTIC, 1, CV_PI/360, 30, 10, 30);


	for(int i = 0; i < MIN(lines_->total,20); i++){
	//for (int i = 0; i < MIN(lines_->total,15); i++) {

		float* line = (float*) cvGetSeqElem(lines_, i);
		rho = line[0];
		theta = line[1];

		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		point1.x = cvRound(x0 + 600 * (-b));
		point1.y = cvRound(y0 + 600 * (a));
		point2.x = cvRound(x0 - 600 * (-b));
		point2.y = cvRound(y0 - 600 * (a));
		point3.x = 300, point3.y = 300;
		point4.x = 300, point4.y = 600;
		point5.x = 300, point5.y = 0;

		//cvLine(rawData_, point1, point2, CV_RGB(255,0,0), 1, CV_AA,0);

		cvLine(rawData_, point3, point4, CV_RGB(0,0,255), 1, CV_AA, 0);
		cvLine(rawData_, point3, point5, CV_RGB(0,0,255), 1, CV_AA, 0);

		if (intersect(point1, point2, point3, point4)) {
			{
				rrhomean += rho;
				rthetamean += theta;
				rmc++;
				//cvLine(workingImg_, point1, point2, CV_RGB(0,0,255), 1, CV_AA,0);
			}
		} else if (intersect(point1, point2, point3, point5)) {
			{
				lrhomean += rho;
				lthetamean += theta;
				lmc++;
				//cvLine(workingImg_, point1, point2, CV_RGB(255,255,255), 1,CV_AA, 0);
			}
		}
	}
	theta = lthetamean / lmc;
	rho = lrhomean / lmc;

	a = cos(theta);
	b = sin(theta);
	x0 = a * rho;
	y0 = b * rho;
	point1.x = cvRound(x0 + 600 * (-b)), point1.y = cvRound(y0 + 600 * (a));
	point2.x = cvRound(x0 - 600 * (-b)), point2.y = cvRound(y0 - 600 * (a));

	//cvLine(rawData_, point1, point2, CV_RGB(255,0,0), 3, CV_AA, 0);

	point4.x = 300;
	point4.y = 300;

	point5.x = point4.x + 800 * sin(CV_PI - (theta - CV_PI / 2));
	point5.y = point4.y + 800 * cos(CV_PI - (theta - CV_PI / 2));
	cvLine(rawData_, point5, point4, CV_RGB(255,255,255), 1, CV_AA, 0);

	rows_.header.stamp = ros::Time::now();
	rows_.leftvalid = false;
	rows_.rightvalid = false;


	//detect valid lines
	if (intersect(point1, point2, point4, point5)) {

		right_distance_ = sqrt(((intersection_.y - 300) * (intersection_.y- 300)) + ((intersection_.x - 300) * (intersection_.x - 300)))* 2;
		right_angle_ = (theta) - CV_PI / 2;
		right_count++;

		right_angle_rolling_mean_[right_count%10] = right_angle_;
		right_dist_rolling_mean_[right_count%10] = right_distance_;
		right_angle_ = 0;
		right_distance_ = 0;

		for(int i=0;i < 10;i++){
			right_angle_ += right_angle_rolling_mean_[i];
			right_distance_ += right_dist_rolling_mean_[i];
		}

		right_angle_ = right_angle_/10;
		right_distance_ = right_distance_ /10;

		ROS_WARN("right_distance_: %f",right_distance_);
		cvLine(rawData_, point1, point2, CV_RGB(0,255,0), 1, CV_AA, 0);


		marker_r.header.frame_id = "/laser_front_link";
		marker_r.header.stamp = ros::Time::now();

		marker_r.ns = "kf_shapes";


		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker_r.type = visualization_msgs::Marker::CUBE;
		// Set the marker action.  Options are ADD and DELETE
		marker_r.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker_r.id = 2;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(right_angle_);
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = -((float) right_distance_) / 100;
		marker_r.pose.position.z = 0;
		marker_r.pose.orientation = pose_quat;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker_r.scale.x = 10.0;
		marker_r.scale.y = 0.1;
		marker_r.scale.z = 0.5;
		marker_r.color.r = 0.0f;
		marker_r.color.g = 1.0f;
		marker_r.color.b = 0.0f;
		marker_r.color.a = 0.5;
		marker_r.lifetime = ros::Duration(.1);
		marker_r_pub.publish(marker_r);

		// Set the color -- be sure to set alpha to something non-zero!

		// Publish the marker

		rows_.rightvalid = true;
		rows_.rightdistance = ((float) right_distance_) / 100;
		rows_.rightangle = right_angle_;

	}else{
		rows_.rightvalid = false;
		rows_.rightdistance = 0;
		rows_.rightangle = 0;

	}


	theta = rthetamean / rmc;
	rho = rrhomean / rmc;

	a = cos(theta);
	b = sin(theta);
	x0 = a * rho;
	y0 = b * rho;
	point1.x = cvRound(x0 + 600 * (-b)), point1.y = cvRound(y0 + 600 * (a));
	point2.x = cvRound(x0 - 600 * (-b)), point2.y = cvRound(y0 - 600 * (a));

	point4.x = 300;
	point4.y = 300;
	point5.x = point4.x - 800 * sin(CV_PI - (theta - CV_PI / 2));
	point5.y = point4.y - 800 * cos(CV_PI - (theta - CV_PI / 2));

	cvLine(rawData_, point5, point4, CV_RGB(255,255,255), 1, CV_AA, 0);

	//detect valid lines
	if (intersect(point1, point2, point4, point5)) {

		left_distance_ = sqrt(((intersection_.y - 300) * (intersection_.y- 300)) + ((intersection_.x - 300) * (intersection_.x - 300)))* 2;
		left_angle_ = (theta) - CV_PI / 2;
		left_count++;

		left_angle_rolling_mean_[left_count%10] = left_angle_;
		left_dist_rolling_mean_[left_count%10] = left_distance_;
		left_angle_ = 0;
		left_distance_ = 0;

		for(int i=0;i < 10;i++){
			left_angle_ += left_angle_rolling_mean_[i];
			left_distance_ += left_dist_rolling_mean_[i];
		}

		left_angle_ = left_angle_/10;
		left_distance_ = left_distance_ /10;


		ROS_WARN("left_distance_: %f",left_distance_);

		marker_r.id = 1;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(left_angle_);

		marker_r.color.r = 0.0f;
		marker_r.color.g = 0.0f;
		marker_r.color.b = 1.0f;
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = ((float) left_distance_) / 100;
		marker_r.pose.position.z = 0;
		marker_r.pose.orientation = pose_quat;
		marker_r_pub.publish(marker_r);

		//rolling_mean_[count%10] = right_angle_;

		//left_angle_ = 0;

		//for(int i=0;i < 10;i++){
		//	left_angle_ += rolling_mean_[i];
		//}

		//right_angle_ = right_angle_/10;

		cvLine(rawData_, point1, point2, CV_RGB(0,255,255), 1, CV_AA, 0);

		rows_.leftvalid = true;
		rows_.leftdistance = ((float) left_distance_) / 100;
		rows_.leftangle = left_angle_;

	}else{
		rows_.leftvalid = false;
		rows_.leftdistance = 0;
		rows_.leftangle = 0;
	}

	if (rows_.leftvalid && rows_.rightvalid){
		e_angle = (rows_.leftangle + rows_.rightangle) / 2;
		e_distance = (rows_.leftdistance - rows_.rightdistance);

		row_dist_est =( (rows_.leftdistance + rows_.rightdistance) + row_dist_est)/2;

		ROS_INFO("2ROWS row_dist_est %f, e_dist %f",row_dist_est,e_distance);

		rows_.error_angle = e_angle;
		rows_.error_distance = e_distance;
		rows_.var = 5^2;

		marker_r.id = 3;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(e_angle);

		marker_r.color.r = 0.0f;
		marker_r.color.g = 1.0f;
		marker_r.color.b = 1.0f;

		marker_r.pose.orientation = pose_quat;
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = (e_distance);
		marker_r.pose.position.z = 0;
		marker_r_pub.publish(marker_r);
	}else if (rows_.leftvalid && !rows_.rightvalid){
		e_angle = (rows_.leftangle);
		e_distance = 0;//row_dist_est/2-(rows_.leftdistance);

		//ROS_INFO("e_angle %f, e_dist %f",e_angle,e_distance);

		rows_.error_angle = e_angle;
		rows_.error_distance = e_distance;//e_distance-(0.75/2);
		rows_.var = 10^2;

		marker_r.id = 3;

		ROS_INFO("LEFTROW row_dist_est %f, e_dist %f",row_dist_est,e_distance);


		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(e_angle);

		marker_r.color.r = 0.0f;
		marker_r.color.g = 1.0f;
		marker_r.color.b = 1.0f;

		marker_r.pose.orientation = pose_quat;
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = (e_distance);
		marker_r.pose.position.z = 0;
		marker_r_pub.publish(marker_r);
	}else if (!rows_.leftvalid && rows_.rightvalid){


		e_angle = (rows_.rightangle);
		e_distance = 0;//row_dist_est/2-(rows_.rightdistance);

		//ROS_INFO("e_angle %f, e_dist %f",e_angle,e_distance);
		ROS_INFO("LRIGHTROW row_dist_est %f, e_dist %f",row_dist_est,e_distance);


		rows_.error_angle = (0.75/2)-e_angle;
		rows_.error_distance = e_distance;//e_distance-(0.75/2);
		rows_.var = 10^2;

		marker_r.id = 3;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(e_angle);

		marker_r.color.r = 0.0f;
		marker_r.color.g = 1.0f;
		marker_r.color.b = 1.0f;

		marker_r.pose.orientation = pose_quat;
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = (e_distance);
		marker_r.pose.position.z = 0;
		marker_r_pub.publish(marker_r);
	}else{
		e_angle = 0;
		e_distance = 0;

		ROS_INFO("e_angle %f, e_dist %f",e_angle,e_distance);
		ROS_INFO("NOROW row_dist_est %f, e_dist %f",row_dist_est,e_distance);

		rows_.error_angle = e_angle;
		rows_.error_distance = e_distance;
		rows_.var = 4000^2;

		marker_r.id = 3;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(e_angle);

		marker_r.color.r = 0.0f;
		marker_r.color.g = 1.0f;
		marker_r.color.b = 1.0f;

		marker_r.pose.orientation = pose_quat;
		marker_r.pose.position.x = 0;
		marker_r.pose.position.y = (e_distance);
		marker_r.pose.position.z = 0;
		marker_r_pub.publish(marker_r);
	}




	//cvLine(rawData_, cvPoint(0,300+150), cvPoint(600,300+150), CV_RGB(255,255,255), 1, CV_AA, 0);
	//cvLine(rawData_, cvPoint(0,300-150), cvPoint(600,300-150), CV_RGB(255,255,255), 1, CV_AA, 0);
	row_pub.publish(rows_);

	//cvShowImage("TEST", rawData_);
	//cvWaitKey(10);
	//pc_pub.publish(cloud);

}

int MaizeDetector::intersect(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4)
/****************************************************************************
 *   Function : See module specification (.h-file).
 ****************************************************************************/
{
	// Store the values for fast access and easy
	// equations-to-code conversion
	float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0)
		return 0;

	// Get the x and y
	float pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
	float x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	float y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < MIN(x2, x1) || x > MAX(x2, x1) || x < MIN(x4, x3) || x
			> MAX(x4, x3))
		return 0;
	if (y < MIN(y2, y1) || y > MAX(y2, y1) || y < MIN(y4, y3) || y
			> MAX(y4, y3))
		return 0;

	intersection_.x = x;
	intersection_.y = y;

	return 1;
}

