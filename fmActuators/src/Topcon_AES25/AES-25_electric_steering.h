#include "fmMsgs/can.h"
#include "ros/ros.h"

union AES25_UInt
{
  unsigned char byte[2];
  unsigned short uInt;
};

//Union used for signed values recieved via CANBus
union AES25_SInt
{
  char byte[2];
  short sInt;
};

// Motor control state
enum
{
  AES_MCS_INIT = 0, AES_MCS_WAIT_FOR_INDEX, AES_MCS_WAIT_FOR_PRECHARGE, AES_MCS_WAIT_FOR_READY, AES_MCS_SET_PID_P,
  AES_MCS_SET_PID_I, AES_MCS_SET_PID_D, AES_MCS_READY
};

enum
{
  AES_SET_PARAM_MAX_SPEED = 0, AES_SET_PARAM_PROPORTIONAL_GAIN, AES_SET_PARAM_INTEGRAL_GAIN,
  AES_SET_PARAM_DERIVATIV_GAIN
};

enum
{
  INDEX_MOTOR_ALARM = 0, INDEX_MOTOR_TEMP, INDEX_INVERTER_TEMP, INDEX_INVERTER_VOLTAGE, INDEX_MOTOR_PROTOCOL_LO,
  INDEX_MOTOR_PROTOCOL_HI, INDEX_MOTOR_FLAGS, INDEX_CAN_TIMEOUT_TICKS
};
#define MOTOR_STARTUP_TICKS   50                 // 1.0 seconds
#define MOTOR_WAIT_TICKS      15                 // 0.3 seconds
#define MOTOR_MIN_TORQUE 0

// Package for Controller status info
#define MAX_CONTROLLER_INFO_ITEMS      8

#define CONTROLLER_CAN_TIMEOUT_TICKS 100*0.02

#define AES_MC_SLAVE_ENABLE         0x01
#define AES_MC_TORQUE_ENABLE        0x02
#define AES_MC_PRECHARGE_ENABLE     0x04
#define AES_MC_MAIN_BREAKER_ENABLE  0x08

#define AES_MS_TORQUE_ON            0x01
#define AES_MS_PRECHARGE_DONE       0x02
#define AES_MS_LIMIT_ALARM          0x04
#define AES_MS_STOP_ALARM           0x08
#define AES_MS_ALIGNED              0x10

class AES25
{
private:

   //unsigned CAN outputs
   AES25_UInt motorSetSpeed, motorSetTorque;
   unsigned char motorControlBits_;
   unsigned char motorSetParameter_;
   //signed CAN outputs
   AES25_SInt motorSetAngle;

   //unsigned CAN inputs
   unsigned char controllerInfo_[MAX_CONTROLLER_INFO_ITEMS];
   //signed CAN inputs
   AES25_SInt motorSpeed, motorTorque, motorAngle;

   //AES-25 State
   unsigned int motorState_;
   //AES-25 Wait timer
   unsigned int motorWaitTimer_;

   fmMsgs::can aes_tx_msg_;

  void resetControllerVariables();
  void updateMotorControl();

public:

  AES25();
  fmMsgs::can processCanTxEvent();
  void processCanRxEvent(const fmMsgs::can::ConstPtr& can_rx_msg);
  bool turnSteeringWheel(unsigned short max_turning_speed, unsigned short max_motor_torque, signed short motor_angle);
};
