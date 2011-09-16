#include <ros/ros.h>
#include "fmMsgs/row.h"
#include "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point32.h"
#include "tf/tf.h"

#include "fmMsgs/row.h"


#define IMU_DT 0.01
#define GYRO_VAR pow(2,2)*M_PI/180 // defined in rad/sec

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

fmMsgs::row kf_row_msg;
ros::Publisher kf_row_pub;

double kf_est_angle_;
double kf_est_var_;

double kf_pri_angle_;
double kf_fri_bar_;

double kf_post_angle_;
double kf_kost_bar_;

double kf_error_distance; // no prediction yes ... pass-trough!

double kf_k_;


void helloKitty(const sensor_msgs::ImuConstPtr & imu_msg){
	//ROS_INFO("IMU FETCHED %f", imu_msg->angular_velocity.z);
	//kalman prediction step
	kf_pri_angle_ = (kf_est_angle_ + imu_msg->angular_velocity.z*IMU_DT);

	if(kf_pri_angle_ >= M_PI *2){
		kf_pri_angle_ -= M_PI*2;
	}else if(kf_pri_angle_ < 0){
		kf_pri_angle_ += M_PI*2;
	}
	kf_fri_bar_ = kf_est_var_+ GYRO_VAR * IMU_DT;

	kf_est_angle_ = kf_pri_angle_;
	kf_est_var_ = kf_fri_bar_;

	ROS_INFO("kf_pri_var: %f , kf_pri_angle: %f",kf_fri_bar_,kf_pri_angle_);


	kf_row_msg.header.stamp = ros::Time::now();
    kf_row_msg.error_angle = kf_est_angle_ -M_PI;
    kf_row_msg.error_distance = kf_error_distance;
    kf_row_msg.var = kf_est_var_;

    kf_row_pub.publish(kf_row_msg);




}

void killAllHumans(const fmMsgs::rowConstPtr & row_msg){
	ROS_INFO("ROW FETCHED, var %f, e_dis %f e_ang %f",row_msg->var, row_msg->error_distance, row_msg->error_angle);
    kf_k_ = kf_fri_bar_ / (kf_fri_bar_ + (row_msg->var));
    kf_post_angle_ = kf_pri_angle_ + kf_k_ * (row_msg->error_angle+M_PI - kf_pri_angle_);
	kf_kost_bar_ = kf_fri_bar_* (1-kf_k_ );

	kf_est_angle_ = kf_post_angle_;
	kf_est_var_ = kf_kost_bar_;

	ROS_INFO("kf_post_var: %f , kf_post_angle: %f",kf_kost_bar_,kf_post_angle_);

	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(kf_post_angle_);
	marker.pose.position.y = ((float) row_msg->error_distance);

	marker.pose.orientation = pose_quat;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side

	kf_error_distance = row_msg->error_distance;

	marker_pub.publish(marker);


}


void init_marker(){
	marker.header.frame_id = "/laser_front_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;
	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.id = 2;

	marker.pose.position.x = 0;
	marker.pose.position.z = 0;

	marker.scale.x = 10.0;
	marker.scale.y = 0.1;
	marker.scale.z = 0.5;
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.5;
	marker.lifetime = ros::Duration(.1);



}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "husmand");



	//init kf -
	kf_est_angle_ = M_PI;
	kf_est_var_ = (pow(20,2)*M_PI/180);
	kf_error_distance = 0;

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	init_marker();

	std::string imu_sub_top_;
	std::string row_sub_top_;

	ros::Subscriber imu_sub_;
	ros::Subscriber row_sub_;


	n.param<std::string>("imu_sub_top", imu_sub_top_, "/vic_sensors/imu_parser/imu_msg");
	n.param<std::string>("row_sub", row_sub_top_, "/rows");

	imu_sub_ = nh.subscribe<sensor_msgs::Imu>(imu_sub_top_.c_str(),1, &helloKitty );
	row_sub_ = nh.subscribe<fmMsgs::row>(row_sub_top_.c_str(),1, &killAllHumans);

	kf_row_pub = nh.advertise<fmMsgs::row>("kalman_row_estimate",1);

	marker_pub = nh.advertise<visualization_msgs::Marker>("kalan_marker", 1);

	ros::spin();
}
