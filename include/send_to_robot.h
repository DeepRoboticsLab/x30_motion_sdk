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


#ifndef X30_SEND_TO_ROBOT_H_
#define X30_SEND_TO_ROBOT_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include "command_list.h"
#include "udp_socket.hpp"
#include "x30_types.h"

#define DEFAULT_ROBOT_IP "192.168.1.103"
#define DEFAULT_ROBOT_PORT 43893

#define ABLE 2
#define UNABLE 1

namespace x30{
/**
 * @brief A class used to send robot data.
 */
class SendToRobot{
  private:
    // RobotCmdSDK robot_cmd_; /**< View robot_tppes.h.*/
    UDPSocket *udp_socket_; /**< What udpSocket needs*/

    /**
    * @brief Send instructions.
    * @param Command
    */
    void cmd_done(Command& command);
  public:
    /// @brief Constructor with IP and port parameters.
    /// @param[in] ip The IP address to send data to.
    /// @param[in] port The port number to send data to.
    SendToRobot(std::string ip = DEFAULT_ROBOT_IP, uint16_t port = DEFAULT_ROBOT_PORT);
    /**
    * @brief Create the send thread using the work()
    */
    void start_work();

    /**
    * @brief Send data to robot.
    */   
    void work();

    // /**
    // * @brief Initialization timer for receiving data
    // */ 
    // void init(void);

    /**
    * @brief Send the set joint target status to the robot.Input:RobotCmdSDK
    * @param RobotCmdSDK
    */ 
    void set_send(RobotCmdSDK&);

    /**
    * @brief Initialize the robot for the first time after powering on
    */ 
    void robot_state_init(void);

    /**
    * @brief Set the command code and command value to send
    * @param code
    * @param value
    */ 
    void set_cmd(uint32_t code , uint32_t value);

    /**
    * @brief Select the control right
    * @param  1:Original robot algorithm control  2: SDK control.
    */ 
    void control_get(int mode){ 
      if(mode == UNABLE){
        RobotCmdSDK cmd;
        for(int i = 0; i < 12; i++){
          cmd.joint_cmd[i].pos = 0.0;
          cmd.joint_cmd[i].tor = 0.0;
          cmd.joint_cmd[i].vel = 0.0;
          cmd.joint_cmd[i].kp = 0.0;
          cmd.joint_cmd[i].kd = 12.5;
        }
        set_send(cmd);
        sleep(5);
        Command command_temp(0x0113,0, 0);
        cmd_done(command_temp);
      }else if (mode == ABLE){
        Command command_temp(0x0114,0, 0);
        cmd_done(command_temp);
      }
    }// void control_get(void);
};

/**
 * @brief A class used to get time.
 */
class TimeTool{
  private:
    int tfd;    /**< Timer descriptor.*/
    int efd;    /**< Epoll descriptor.*/
    int fds, ret; /**< Variables used to initialize the timer.*/
    uint64_t value; /**< Variables used to initialize the timer.*/
    struct epoll_event ev, *evptr; /**< Variables used to initialize the timer.*/
    struct itimerspec time_intv;  /**< Variables used to initialize the timer.*/
  public:
    timespec system_time; /**< A class for accurately obtaining time.*/
    /**
    * @brief Initialize timer, input cycle(ms).
    * @param Cycle time unit: ms
    */ 
    void time_init(int ms); 

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */ 
    int time_interrupt(); /**< Acquire interrupt signal.*/

    /**
    * @brief How long has it been
    * @param Initial time
    */ 
    double get_now_time(double start_time);

    /**
    * @brief Get current time
    */ 
    double get_start_time(); /**< Get start time.*/
};

};//namespace x30
#endif  // PARSE_CMD_H_
