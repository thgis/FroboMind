#include "AES-25_electric_steering.h"

void AES25::resetControllerVariables()
{
  motorState_ = AES_MCS_INIT;
  motorWaitTimer_ = MOTOR_STARTUP_TICKS;

  motorSpeed.sInt = 0;
  motorTorque.sInt = 0;
  motorAngle.sInt = 0;

  motorSetTorque.uInt = MOTOR_MIN_TORQUE;
  motorSetSpeed.uInt = 0;
  motorSetAngle.sInt = 0;
  motorSetParameter_ = AES_SET_PARAM_MAX_SPEED;
  motorControlBits_ = 0;

  controllerInfo_[INDEX_MOTOR_FLAGS] = 0;
  controllerInfo_[INDEX_MOTOR_ALARM] = 0;
  controllerInfo_[INDEX_MOTOR_TEMP] = 0;
  controllerInfo_[INDEX_INVERTER_TEMP] = 0;
  controllerInfo_[INDEX_INVERTER_VOLTAGE] = 0;
}

void AES25::updateMotorControl()
{
  switch (motorState_)
  {
    case AES_MCS_INIT:
      if (motorWaitTimer_ == 0)
      {
        motorControlBits_ = AES_MC_SLAVE_ENABLE;
        motorState_ = AES_MCS_WAIT_FOR_INDEX;
        ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = AES_MCS_WAIT_FOR_INDEX");
      }
      else
      {
        motorWaitTimer_--;
      }
      break;
    case AES_MCS_WAIT_FOR_INDEX:
      // When align bit is set we can proceed.
      if (controllerInfo_[INDEX_MOTOR_FLAGS] & AES_MS_ALIGNED)
      {
        motorControlBits_ |= AES_MC_PRECHARGE_ENABLE;
        motorState_ = AES_MCS_WAIT_FOR_PRECHARGE;
        ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = AES_MCS_WAIT_FOR_PRECHARGE");
      }
      break;
    case AES_MCS_WAIT_FOR_PRECHARGE:
      // When precharge bit is set we can proceed.
      if (controllerInfo_[INDEX_MOTOR_FLAGS] & AES_MS_PRECHARGE_DONE)
      {
        motorControlBits_ |= AES_MC_MAIN_BREAKER_ENABLE;
        motorWaitTimer_ = MOTOR_WAIT_TICKS;
        motorState_ = AES_MCS_SET_PID_P;
        ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = motorState_ = AES_MCS_SET_PID_P;");
      }
      break;
    case AES_MCS_SET_PID_P:
      motorSetParameter_ = AES_SET_PARAM_PROPORTIONAL_GAIN;
      motorSetSpeed.uInt = 50;
      motorState_ = AES_MCS_SET_PID_I;
      ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = motorState_ = AES_MCS_SET_PID_I;");
      break;
    case AES_MCS_SET_PID_I:
      motorSetParameter_ = AES_SET_PARAM_INTEGRAL_GAIN;
      motorSetSpeed.uInt = 20;
      motorState_ = AES_MCS_SET_PID_D;
      ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = motorState_ = AES_MCS_SET_PID_D;");
      break;
    case AES_MCS_SET_PID_D:
      motorSetParameter_ = AES_SET_PARAM_DERIVATIV_GAIN;
      motorSetSpeed.uInt = 50;
      motorState_ = AES_MCS_WAIT_FOR_READY;
      ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = motorState_ = AES_MCS_WAIT_FOR_READY;");
      break;
    case AES_MCS_WAIT_FOR_READY:
      if (motorWaitTimer_ == 0)
      {
        motorControlBits_ |= AES_MC_TORQUE_ENABLE;
        motorState_ = AES_MCS_READY;
        ROS_DEBUG_NAMED("AES25::updateMotorControl","motorState_ = motorState_ = AES_MCS_READY;");
        motorSetParameter_ = 0;

      }
      else
      {
        motorWaitTimer_--;
      }
      break;
    case AES_MCS_READY:

      break;
  }
}

AES25::AES25()
{
  resetControllerVariables();
}

fmMsgs::can AES25::processCanTxEvent()
{
  static int canToggle = 0;

  updateMotorControl();
  // Alternate between sending 200h and 300h command
  // to get status return.
  if (canToggle == 0)
  {
    aes_tx_msg_.id = 0x200;
    aes_tx_msg_.header.stamp = ros::Time::now();
    aes_tx_msg_.length = 8;
    canToggle++;
  }
  else
  {
    aes_tx_msg_.id = 0x300;
    aes_tx_msg_.header.stamp = ros::Time::now();
    aes_tx_msg_.length = 8;
    canToggle = 0;
  }

  aes_tx_msg_.data[0] = motorSetSpeed.byte[0];
  aes_tx_msg_.data[1] = motorSetSpeed.byte[1];
  aes_tx_msg_.data[2] = motorSetAngle.byte[0];
  aes_tx_msg_.data[3] = motorSetAngle.byte[1];
  aes_tx_msg_.data[4] = motorControlBits_;
  aes_tx_msg_.data[5] = motorSetTorque.byte[0];
  aes_tx_msg_.data[6] = 0;
  aes_tx_msg_.data[7] = motorSetParameter_;

  if (controllerInfo_[INDEX_CAN_TIMEOUT_TICKS] > 0)
  {
    controllerInfo_[INDEX_CAN_TIMEOUT_TICKS]--;
    // Reset Controller if we lose CAN
    if (controllerInfo_[INDEX_CAN_TIMEOUT_TICKS] == 0)
    {
      ROS_ERROR("AES25 watchdog timeout!");
      resetControllerVariables();
    }
  }
  return aes_tx_msg_;
}

void AES25::processCanRxEvent(const fmMsgs::can::ConstPtr& can_rx_msg)
{
  if (can_rx_msg->id == 0x180)
  {
    motorSpeed.byte[0] = can_rx_msg->data[0];
    motorSpeed.byte[1] = can_rx_msg->data[1];
    motorTorque.byte[0] = can_rx_msg->data[2];
    motorTorque.byte[1] = can_rx_msg->data[3];
    motorAngle.byte[0] = can_rx_msg->data[4];
    motorAngle.byte[1] = can_rx_msg->data[5];
    controllerInfo_[INDEX_MOTOR_FLAGS] = can_rx_msg->data[6];
    controllerInfo_[INDEX_CAN_TIMEOUT_TICKS] = CONTROLLER_CAN_TIMEOUT_TICKS;

    ROS_DEBUG_THROTTLE_NAMED(1, "AES25 0x180", "Motorspeed: %d MotorTorque: %d Motorangle: %d MotorFlags: %x", motorSpeed.sInt, motorTorque.sInt,motorAngle.sInt,controllerInfo_[INDEX_MOTOR_FLAGS]);

  }
  else if (can_rx_msg->id == 0x280)
  {
    controllerInfo_[INDEX_MOTOR_ALARM] = can_rx_msg->data[0];
    controllerInfo_[INDEX_MOTOR_TEMP] = can_rx_msg->data[1];
    controllerInfo_[INDEX_INVERTER_TEMP] = can_rx_msg->data[2];
    controllerInfo_[INDEX_INVERTER_VOLTAGE] = can_rx_msg->data[3];
    controllerInfo_[INDEX_MOTOR_PROTOCOL_LO] = can_rx_msg->data[4];
    controllerInfo_[INDEX_MOTOR_PROTOCOL_HI] = can_rx_msg->data[5];
    controllerInfo_[INDEX_CAN_TIMEOUT_TICKS] = CONTROLLER_CAN_TIMEOUT_TICKS;

    ROS_DEBUG_COND_NAMED(controllerInfo_[INDEX_MOTOR_ALARM], "AES25 0x280", "MotorAlarm: %d MotorTemp: %d InverterTemp: %d InverterVoltage: %d MotorProtocolVersionLo: %d MotorProtocolVersionHi: %d", controllerInfo_[INDEX_MOTOR_ALARM], controllerInfo_[INDEX_MOTOR_TEMP], controllerInfo_[INDEX_INVERTER_TEMP], controllerInfo_[INDEX_INVERTER_VOLTAGE], controllerInfo_[INDEX_MOTOR_PROTOCOL_LO], controllerInfo_[INDEX_MOTOR_PROTOCOL_HI]);

  }
  else if (can_rx_msg->id == 0x700)
  {
    ROS_DEBUG_THROTTLE_NAMED(1, "AES25 0x700","Undescribed CANbus msg!");
  }
}

bool AES25::turnSteeringWheel(unsigned short max_turning_speed, unsigned short max_motor_torque, signed short motor_angle)
{
  if (motorState_ == AES_MCS_READY)
  {
    motorSetSpeed.uInt = max_turning_speed;
    motorSetAngle.sInt = motor_angle;
    motorSetTorque.uInt = max_motor_torque;
    return true;
  }
  return false;
}

