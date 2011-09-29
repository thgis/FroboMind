#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/gpgga.h"
#include "fmMsgs/gps_state.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>

ros::Publisher gps_state_pub;
std::string frame_id;
fmMsgs::gps_state gps_state_msg;


#define deg_rad      0.01745329251994 /* 2pi/360 */

/***************************************************************************/
void latlon2utm (double lat, double lon,
	int *znum, char *zlet, double *n, double *e)
{
	double flat = 1/298.257223563;	/* WGS84 flat */
	double a = 6378137;				/* WGS84 equatorial radius */
	double k0 = 0.9996;
	double latr = lat * deg_rad;
	double lonr = lon * deg_rad;
	double lonr_center;
	double es;						/* eccentricity^2 */
	double eps;
	double N, T, C, A, M;

	/* test if the UTM projection is defined for this latitude and longitude */
	if (lat <= 84 && lat >= -80)
	{
		/* determine the UTM zone number */
		*znum = (int) ((lon + 180)/6) + 1;

		if( lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0 )
			*znum = 32;

	  	/* Take care of zone numbers for Svalbard */
		if( lat >= 72.0 && lat < 84.0 )
		{
		  if(lon >=  0.0 && lon <  9.0) *znum = 31;
		  else if( lon >=  9.0 && lon < 21.0 ) *znum = 33;
		  else if( lon >= 21.0 && lon < 33.0 ) *znum = 35;
		  else if( lon >= 33.0 && lon < 42.0 ) *znum = 37;
		}

		/* determine the UTM zone letter */
		if     (( 84.0 >= lat) && (lat >=  72.0)) *zlet = 'X';
		else if(( 72.0 >  lat) && (lat >=  64.0)) *zlet = 'W';
		else if(( 64.0 >  lat) && (lat >=  56.0)) *zlet = 'V';
		else if(( 56.0 >  lat) && (lat >=  48.0)) *zlet = 'U';
		else if(( 48.0 >  lat) && (lat >=  40.0)) *zlet = 'T';
		else if(( 40.0 >  lat) && (lat >=  32.0)) *zlet = 'S';
		else if(( 32.0 >  lat) && (lat >=  24.0)) *zlet = 'R';
		else if(( 24.0 >  lat) && (lat >=  16.0)) *zlet = 'Q';
		else if(( 16.0 >  lat) && (lat >=   8.0)) *zlet = 'P';
		else if((  8.0 >  lat) && (lat >=   0.0)) *zlet = 'N';
		else if((  0.0 >  lat) && (lat >=  -8.0)) *zlet = 'M';
		else if(( -8.0 >  lat) && (lat >= -16.0)) *zlet = 'L';
		else if((-16.0 >  lat) && (lat >= -24.0)) *zlet = 'K';
		else if((-24.0 >  lat) && (lat >= -32.0)) *zlet = 'J';
		else if((-32.0 >  lat) && (lat >= -40.0)) *zlet = 'H';
		else if((-40.0 >  lat) && (lat >= -48.0)) *zlet = 'G';
		else if((-48.0 >  lat) && (lat >= -56.0)) *zlet = 'F';
		else if((-56.0 >  lat) && (lat >= -64.0)) *zlet = 'E';
		else if((-64.0 >  lat) && (lat >= -72.0)) *zlet = 'D';
		else if((-72.0 >  lat) && (lat >= -80.0)) *zlet = 'C';

		/* calculate UTM northing and easting */
		es = 2*flat-flat*flat;
		eps = (es)/(1-es);

		/* find the center longitude for the UTM zone */
		lonr_center = ((*znum -1)*6-180+3) * deg_rad;

		N = a/sqrt(1-es*sin(latr)*sin(latr));
		T = tan(latr)*tan(latr);
		C = eps*cos(latr)*cos(latr);
		A = cos(latr)*(lonr-lonr_center);

		M = a*((1 - es/4 - 3*es*es/64 - 5*es*es*es/ 256)*latr
			- (3* es/8 + 3*es*es/32 + 45*es*es*es/1024)*sin(2*latr)
			+ (15*es*es/256 + 45*es*es*es/1024)*sin(4*latr)
			-(35*es*es*es/3072)*sin(6*latr));

		*e = (k0*N*(A+(1-T+C)*A*A*A/6
			+ (5-18*T+T*T+72*C-58*eps)*A*A*A*A*A/120)
			+ 500000.0);

		*n = (k0*(M+N*tan(latr)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			+ (61-58*T+T*T+600*C-330*eps)*A*A*A*A*A*A/720)));

		if(lat < 0)
			*n += 10000000.0; /* 10000000 meter offset for southern hemisphere */
	}
	else
      *znum = -1;
}



void gpsStateCallback(const fmMsgs::gpgga::ConstPtr& msg)
{
	// save current time
	gps_state_msg.header.stamp = ros::Time::now();

	// import data from the gpgga topic
	gps_state_msg.time_recv = msg->header.stamp;
	gps_state_msg.fix = msg->fix;
	gps_state_msg.sat = msg->sat;

	if (msg->fix >= 1) // if the GPS currently has a valid satellite fix
	{
		gps_state_msg.time = msg->time;
		gps_state_msg.lat = msg->lat;
		gps_state_msg.lon = msg->lon;
		gps_state_msg.alt = msg->alt;
		gps_state_msg.hdop = msg->hdop;			
		gps_state_msg.geoid_height = msg->geoid_height;

		// convert lat/lon to UTM coordinates
		double n, e;
		int znum;
		char zlet;
		
		latlon2utm (gps_state_msg.lat, gps_state_msg.lon, &znum, &zlet, &n, &e);
		gps_state_msg.utm_zone_num = znum;
		gps_state_msg.utm_zone_let = zlet;
		gps_state_msg.utm_n = n;
		gps_state_msg.utm_e = e;
	}
	gps_state_pub.publish (gps_state_msg); // publish the message
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_state");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "fmSensors/gpgga_msg");
	n.param<std::string> ("publish_topic_id", publish_topic_id, "fmExtractors/gps_state_msg");
//	n.param<std::string> ("frame_id", frame_id, "/base");

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 1, gpsStateCallback);
	gps_state_pub = n.advertise<fmMsgs::gps_state> (publish_topic_id, 1);

	ros::spin();
	return 0;
}

