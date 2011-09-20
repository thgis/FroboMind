#include "LindeLiendeEncoder.h"

void LLENCODER::resetControllerVariables() {
	encoder_state_ = LL_ECS_SEND_1536;
	wt_timer_ = ENCODER_CAN_TIMEOUT_TICKS;
	encoder_pos_.uInt = 0;
	encoder_pos_prev_.uInt = 0;
	//encoder_data_.data = 0;
}

bool LLENCODER::updateEncoderControl() {

	bool send_can_msg = false;

	switch (encoder_state_) {
	case LL_ECS_SEND_1536:
		encoder_data_.id = 1536 + enc_id;
		encoder_data_.data[0] = 0x22;
		encoder_data_.data[1] = 0x0;
		encoder_data_.data[2] = 0x62;
		encoder_data_.data[3] = 0x0;
		encoder_data_.data[4] = 0x0A;
		encoder_data_.data[5] = 0x0;
		encoder_data_.data[6] = 0x0;
		encoder_data_.data[7] = 0x0;
		send_can_msg = true;
		encoder_state_ = LL_ECS_WAIT_1408;
		break;
	case LL_ECS_WAIT_1408:
		break;
	case LL_ECS_SEND_0:
		encoder_data_.id = 0;
		encoder_data_.data[0] = 0x01;
		encoder_data_.data[1] = enc_id;
		encoder_data_.data[2] = 0x0;
		encoder_data_.data[3] = 0x0;
		encoder_data_.data[4] = 0x0;
		encoder_data_.data[5] = 0x0;
		encoder_data_.data[6] = 0x0;
		encoder_data_.data[7] = 0x0;
		send_can_msg = true;
		break;
	case LL_ECS_WAIT_384:
		break;
	case LL_ECS_READY:
		break;
	}

	return send_can_msg;
}

LLENCODER::LLENCODER() {
	resetControllerVariables();
}

void LLENCODER::processCanTxEvent() {

	static int canToggle = 0;

	if (updateEncoderControl()==true) {

		llenc_tx_msg_.length = 8;
		llenc_tx_msg_.id = encoder_data_.id;
		llenc_tx_msg_.header.stamp = ros::Time::now();
		llenc_tx_msg_.data[0] = encoder_data_.data[0];
		llenc_tx_msg_.data[1] = encoder_data_.data[1];
		llenc_tx_msg_.data[2] = encoder_data_.data[2];
		llenc_tx_msg_.data[3] = encoder_data_.data[3];
		llenc_tx_msg_.data[4] = encoder_data_.data[4];
		llenc_tx_msg_.data[5] = encoder_data_.data[5];
		llenc_tx_msg_.data[6] = encoder_data_.data[6];
		llenc_tx_msg_.data[7] = encoder_data_.data[7];

		can_tx_pub.publish(llenc_tx_msg_);
	}
}

void LLENCODER::processCanRxEvent(const vic_msgs::can::ConstPtr& can_rx_msg) {

	if (can_rx_msg->id == 0x700 + enc_id) {

		wt_timer_ = ENCODER_CAN_TIMEOUT_TICKS;

		encoder_state_ = LL_ECS_SEND_1536;
		processCanTxEvent();
	} else if (can_rx_msg->id == 1408 + enc_id) {

		wt_timer_ = ENCODER_CAN_TIMEOUT_TICKS;

		encoder_state_ = LL_ECS_SEND_0;
		processCanTxEvent();
	} else if (can_rx_msg->id == 384 + enc_id) {

		wt_timer_ = ENCODER_CAN_TIMEOUT_TICKS;

		encoder_pos_.byte[0] = can_rx_msg->data[0];
		encoder_pos_.byte[1] = can_rx_msg->data[1];

		ROS_DEBUG("ID:%d data %d ",can_rx_msg->id,(encoder_pos_.uInt-encoder_pos_prev_.uInt));

		encoder_msg_.header = can_rx_msg->header;
		encoder_msg_.encoderticks = (encoder_pos_.uInt-encoder_pos_prev_.uInt);

		enc_ticks_pub.publish(encoder_msg_);

		encoder_pos_prev_.uInt = encoder_pos_.uInt;
	}
}

void LLENCODER::watchdog(){

		wt_timer_--;;
		if (wt_timer_== 0) {
			ROS_ERROR("Encoder watchdog timeout!");
			resetControllerVariables();
			processCanTxEvent();
		}
}

