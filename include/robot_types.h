/**
 * @file robot_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef ROBOT_TYPES_H_
#define ROBOT_TYPES_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>


struct ImuDataSDK{
  int32_t timestamp;
  union{
    float buffer_float[9];
    uint8_t buffer_byte[3][12];
    struct{
      float roll, pitch, yaw;
      float omega_x, omega_y, omega_z;
      float acc_x, acc_y, acc_z;
    };
  };
};

typedef struct{
  float pos;
  float vel;
  float tor;
  float temperature;
}SingleJointData;

struct JointDataSDK{
  union{
    SingleJointData joint_data[12];
    struct {
      SingleJointData fl_leg[3];
      SingleJointData fr_leg[3];
      SingleJointData hl_leg[3];
      SingleJointData hr_leg[3];
    };
  };
};

typedef struct{
  float pos;
  float vel;
  float tor;
  float kp;
  float kd;
}SingleJointCmd;

struct RobotCmdSDK{
  union{
    SingleJointCmd joint_cmd[12];
    struct {
      SingleJointCmd fl_leg[3];
      SingleJointCmd fr_leg[3];
      SingleJointCmd hl_leg[3];
      SingleJointCmd hr_leg[3];
    };
  };
};


typedef struct{
  uint32_t tick;
  ImuDataSDK imu;
  JointDataSDK joint_state;
  union{
    struct{
      float fl_force[3];
      float fr_force[3];
      float hl_force[3];
      float hr_force[3];
    };
    float contact_force[12];
  };
}RobotDataSDK;







#endif  // ROBOT_TYPES_H_