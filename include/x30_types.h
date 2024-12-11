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


#ifndef X30_ROBOT_TYPES_H_
#define X30_ROBOT_TYPES_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>

namespace x30{
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

// struct JointDataSDK{
//   union{
//     std::array<SingleJointData, 12> joint_data;
//     struct {
//       std::array<SingleJointData, 3> fl_leg;
//       std::array<SingleJointData, 3> fr_leg;
//       std::array<SingleJointData, 3> hl_leg;
//       std::array<SingleJointData, 3> hr_leg;
//     };
//   };
// };

typedef struct{
  float pos;
  float vel;
  float tor;
  float kp;
  float kd;
}SingleJointCmd;

struct RobotCmdSDK{
  union{
    std::array<SingleJointCmd, 12> joint_cmd;
    struct {
      std::array<SingleJointCmd, 3> fl_leg;
      std::array<SingleJointCmd, 3> fr_leg;
      std::array<SingleJointCmd, 3> hl_leg;
      std::array<SingleJointCmd, 3> hr_leg;
    };
  };
};


typedef struct{
  uint32_t tick;
  ImuDataSDK imu;
  union{
    std::array<SingleJointData, 12> joint_data;
    struct {
      std::array<SingleJointData, 3> fl_leg;
      std::array<SingleJointData, 3> fr_leg;
      std::array<SingleJointData, 3> hl_leg;
      std::array<SingleJointData, 3> hr_leg;
    };
  };
  union{
    struct{
      std::array<float, 3> fl_force;
      std::array<float, 3> fr_force;
      std::array<float, 3> hl_force;
      std::array<float, 3> hr_force;
    };
    std::array<float, 12> contact_force;
  };
}RobotDataSDK;


};




#endif  // ROBOT_TYPES_H_