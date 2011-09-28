#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/gpgga.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>

#define DEG2RAD M_PI/180.0

ros::Publisher gpgga_pub;
std::string frame_id;

fmMsgs::gpgga gpgga_msg;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

double nmea_to_deg(double pos, std::string dir) {
	pos = pos / 100;
	int dd = floor(pos);
	double mm = (pos - dd) * 100;
	double res = dd + (mm / 60);
	if (dir == "S" || dir == "W") {
		res = 0 - res;
	}
	return res;
}

void gpsCallback(const fmMsgs::serial::ConstPtr& msg)
{
	std::string nmeadata = msg->data;
	boost::replace_all(nmeadata, ",,", ", ,"); // needed to not drop any tokens.
	int count = 0;
 	boost::char_separator<char> sep(",");
	tokenizer::iterator tok_iter; 
	tokenizer tokens(nmeadata, sep);

	// count the number of tokens
//	for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
// 		count ++;
//	}
	try {
		tok_iter = tokens.begin();
		if (tok_iter != tokens.end())
		{
			std::string nmea_id = boost::lexical_cast<std::string>(*tok_iter++);
			if (nmea_id == "$GPGGA") {  
				double nmea_lat;
				double nmea_lon;
				std::string nmea_lat_hem;
				std::string nmea_lon_hem;
//				ROS_INFO("GPGGA received");
				gpgga_msg.header.stamp = ros::Time::now();
				gpgga_msg.time = boost::lexical_cast<std::string>(*tok_iter++);
				nmea_lat = boost::lexical_cast<double>(*tok_iter++);
				nmea_lat_hem = boost::lexical_cast<std::string>(*tok_iter++);
				nmea_lon = boost::lexical_cast<double>(*tok_iter++);
				nmea_lon_hem = boost::lexical_cast<std::string>(*tok_iter++);
				gpgga_msg.fix = boost::lexical_cast<int>(*tok_iter++);
				gpgga_msg.sat = boost::lexical_cast<int>(*tok_iter++);
				gpgga_msg.hdop = boost::lexical_cast<double>(*tok_iter++);

				gpgga_msg.lat = nmea_to_deg (nmea_lat, nmea_lat_hem);
				gpgga_msg.lon = nmea_to_deg (nmea_lon, nmea_lon_hem);

				gpgga_pub.publish(gpgga_msg);
			}
		}
		else {
			ROS_INFO("tok_iter problem");
		}
	}
	catch(boost::bad_lexical_cast &) {
  		ROS_WARN(("Bad lexical cast in GPS Parser! : " + msg->data).c_str());
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_parser");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  std::string subscribe_topic_id;
  std::string publish_topic_id;

  n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "fmBSP/gps_msg");
  n.param<std::string> ("publish_topic_id", publish_topic_id, "gpgga_msg");
//  n.param<std::string> ("frame_id", frame_id, "/base");

  ros::Subscriber sub = n.subscribe(subscribe_topic_id, 1, gpsCallback);
  gpgga_pub = n.advertise<fmMsgs::gpgga> (publish_topic_id, 1);

  ros::spin();

  return 0;
}

