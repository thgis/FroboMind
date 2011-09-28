#include "isb_pseudo_ts.h"

void ISBTS::resetControllerVariables() {

	std::fstream bootcountfile;
	std::string bootcountstring;

	bootcountfile.open("/boot.cnt");

	if (bootcountfile.is_open()) {

		getline(bootcountfile, bootcountstring);

		ts_bootcount_.uShort = atoi(bootcountstring.c_str());
		ts_bootcount_.uShort++;

		bootcountfile.close();

		bootcountfile.open("/boot.cnt");
		bootcountfile << (ts_bootcount_.uShort % 65000) << std::endl;

		//function to close the file:
		bootcountfile.close();

	} else {
		//is_open() returned false and there is a problem:
		ROS_INFO("Can't open the file!");
	}
	ts_count_.uLong = 0;
}

void ISBTS::tsUpdate() {
	ts_count_.uLong++;
	ROS_INFO_THROTTLE(1,"count %d AND as on canbus %d %d %d %d bootcount %d CB %d %d", ts_count_.uLong , ts_count_.byte[0], ts_count_.byte[1], ts_count_.byte[2], ts_count_.byte[3],ts_bootcount_.uShort,ts_bootcount_.byte[0],ts_bootcount_.byte[1]);
	processCanTxEvent();
}

ISBTS::ISBTS() {
	resetControllerVariables();
}

void ISBTS::processCanTxEvent() {

	static bool reset_timer = false;

	ts_can_msg.id = ISB_TS_CAN_ID;
	ts_can_msg.header.stamp = ros::Time::now();
	ts_can_msg.length = 8;
	ts_can_msg.data[0] = ts_count_.byte[0];
	ts_can_msg.data[1] = ts_count_.byte[1];
	ts_can_msg.data[2] = ts_count_.byte[2];
	ts_can_msg.data[3] = ts_count_.byte[3];
	ts_can_msg.data[4] = ts_bootcount_.byte[0];
	ts_can_msg.data[5] = ts_bootcount_.byte[1];

	ros::param::get("~logstate", logstate_);
	ros::param::get("~resettimer", reset_timer);

	if (logstate_) {
		if (reset_timer) {
			ts_can_msg.data[6] = 3;
			ros::param::set("~resettimer", false);
		} else {
			ts_can_msg.data[6] = 1;
		}
	} else {
		if (reset_timer) {
			ts_can_msg.data[6] = 2;
			ros::param::set("~resettimer", false);
		} else {
			ts_can_msg.data[6] = 0;
		}
	}

	//Not used!
	ts_can_msg.data[7] = 0;

	can_tx_pub.publish(ts_can_msg);

}

void ISBTS::processCanRxEvent(const fmMsgs::can::ConstPtr& can_rx_msg) {

}

