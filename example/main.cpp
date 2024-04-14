/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "udp_socket.hpp"
#include "udp_server.hpp"
#include "command_list.h"
#include "parse_cmd.h"
#include "send_to_robot.h"
#include "motion_sdk_example.h"

#include <iostream>
#include <time.h>
#include <string.h>

using namespace std;
  bool is_message_updated_ = false; ///< Flag to check if message has been updated
  /**
   * @brief Callback function to set message update flag
   * 
   * @param code The code indicating the type of message received
   */
  void OnMessageUpdate(uint32_t code){
    if(code == 0x0906){
      is_message_updated_ = true;
    }
  }



int main(int argc, char* argv[]){
  double now_time,start_time;
  RobotCmdSDK robot_joint_cmd;
  TimeTool my_set_timer;
  memset(&robot_joint_cmd, 0, sizeof(robot_joint_cmd));
  SendToRobot* send2robot_cmd = new SendToRobot();                      ///< Create send thread
  ParseCommand* robot_data_rec = new ParseCommand;                         ///< Create a receive resolution thread 
  robot_data_rec->RegisterCallBack(OnMessageUpdate);
  MotionSDKExample robot_set_up_demo;                                    ///< Demos for testing can be deleted by yourself
  RobotDataSDK *robot_data = &robot_data_rec->getRecvState();

  robot_data_rec->startWork();
  my_set_timer.time_init(1);                                          ///< Timer initialization, input: cycle; Unit: ms

  send2robot_cmd->robot_state_init();                                 ///< Return all joints to zero and gain control

  start_time = my_set_timer.get_start_time();                         ///< Obtain time for algorithm usage
  robot_set_up_demo.GetInitData(*robot_data, 0.000);       ///< Obtain all joint states once before each stage (action)
  
/********************************************************/
  int time_tick = 0;
  int loopcount=0;

  while(1){
    if (my_set_timer.time_interrupt() == 1){                          ///< Time interrupt flag, return 1, cycle not reached, return 0, reach a cycle
      continue;
    }
    now_time = my_set_timer.get_now_time(start_time);                ///< Get the current time
    if(robot_data_rec->getDataRevState()==0){                        //No data received from robot
      cout<<" No data from the robot was received! "<<endl;
      break;
    } else {
      robot_data_rec->printData();
    }
    

/*******A simple demo that stands up (for testing and can be deleted by yourself)*********/
    time_tick++;
    if(time_tick < 5000){
      robot_set_up_demo.PreStandUp(robot_joint_cmd, now_time, *robot_data);///< Stand up and prepare for action
    } 
    if(time_tick == 5000){
      robot_set_up_demo.GetInitData(*robot_data, now_time);///< Obtain all joint states once before each stage (action)
    }
    if(time_tick >= 5000 ){
      robot_set_up_demo.StandUp(robot_joint_cmd, now_time, *robot_data);///< Full stand up
    }
    if(time_tick >= 10000){
      send2robot_cmd->control_get(1);                                 ///< Return the control right, input: 1. Original algorithm control of the robot 2. SDK control PS: over 5ms, no data set sent_ Send (cmd), you will lose control, you need to resend to obtain control
      exit(1);
    }


/*********A simple demo that stands up (for testing and can be deleted by yourself)*******/
    // if(is_message_updated_){
    //   send2robot_cmd->set_send(robot_joint_cmd);  
    // }               
    loopcount++;
    // cout<<robot_data->joint_state.joint_data[2].tor<<" | "<<robot_joint_cmd.joint_cmd[2].tor <<" | "<<robot_joint_cmd.fl_leg[2].tor <<endl;
   // cout << robot_joint_cmd.joint_cmd[0].pos<<" "<<robot_joint_cmd.joint_cmd[1].pos<<"  "<<robot_joint_cmd.joint_cmd[2].pos<<"  "<<robot_data->joint_state.fl_leg[0].pos<<"  "<<robot_data->joint_state.fl_leg[1].pos<<"  "<<robot_data->joint_state.fl_leg[2].pos<<" "<< robot_joint_cmd.joint_cmd[3].pos<<" "<<robot_joint_cmd.joint_cmd[4].pos<<"  "<<robot_joint_cmd.joint_cmd[5].pos<<" "<< robot_joint_cmd.joint_cmd[6].pos<<" "<<robot_joint_cmd.joint_cmd[7].pos<<"  "<<robot_joint_cmd.joint_cmd[8].pos<<" "<< robot_joint_cmd.joint_cmd[9].pos<<" "<<robot_joint_cmd.joint_cmd[10].pos<<"  "<<robot_joint_cmd.joint_cmd[11].pos<<" "<<robot_data->imu.acc_x<<"  "<<loopcount << endl;
  }
  return 0;
} 
