#include "fmMsgs/can.h"
#include "fmMsgs/encoder.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"


#include <iostream>
#include <string>
#include <fstream>


#ifndef ISBTS_H_
#define ISBTS_H_

#define ISBTS_CAN_TIMEOUT_TICKS 100
#define ISB_TS_CAN_ID 0x05

union ISBTS_uLong {
	unsigned char byte[4];
	unsigned long uLong;
};

union ISBTS_uShort {
	unsigned char byte[2];
	unsigned short uShort;
};

class ISBTS {
private:

	ISBTS_uLong ts_count_;
	ISBTS_uShort ts_bootcount_;

	fmMsgs::can ts_can_msg;

	void resetControllerVariables();

public:

	bool logstate_;

	ros::Publisher can_tx_pub;

	ISBTS();
	void tsUpdate();
	void processCanTxEvent();
	void processCanRxEvent(const fmMsgs::can::ConstPtr& can_rx_msg);
};

#endif
