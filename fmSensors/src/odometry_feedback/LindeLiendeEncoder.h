#include "vic_msgs/can.h"
#include "vic_msgs/encoder.h"
#include "ros/ros.h"

#ifndef ENCODERDRIVER_H_
#define ENCODERDRIVER_H_

#define ENCODER_CAN_TIMEOUT_TICKS 100

union LLENC_uInt
{
  unsigned char byte[2];
  unsigned short uInt;
};

//Union used for signed values recieved via CANBus
union LLENC_sInt
{
  char byte[2];
  short sInt;
};

// Encoder control state
enum
{
  LL_ECS_INIT = 0, LL_ECS_SEND_1536, LL_ECS_WAIT_1408, LL_ECS_SEND_0 , LL_ECS_WAIT_384, LL_ECS_READY
};

class LLENCODER
{
private:

   int wt_timer_;

   vic_msgs::can encoder_data_;
   vic_msgs::can llenc_tx_msg_;
   vic_msgs::encoder encoder_msg_;

   LLENC_uInt encoder_pos_;
   LLENC_uInt encoder_pos_prev_;

   void resetControllerVariables();
   bool updateEncoderControl();

public:

  int enc_id;
  unsigned int encoder_state_;
  ros::Publisher can_tx_pub;
  ros::Publisher enc_ticks_pub;

  ros::Subscriber can_rx_sub;

  LLENCODER();
  void processCanTxEvent();
  void watchdog();
  void processCanRxEvent(const vic_msgs::can::ConstPtr& can_rx_msg);
};

#endif
