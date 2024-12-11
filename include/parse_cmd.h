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


#ifndef X30_PARSE_CMD_H_
#define X30_PARSE_CMD_H_

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

#include "x30_types.h"
#include "udp_server.hpp"
#define LOCAL_PORT 43897 /**< 43897 Local Port Number.*/
#define STATE_RECEIVE_CODE 0x0906 /**< Command code for receiving robot data.*/


namespace x30{
/**
 * @brief A class used to receive robot data.
 */
class ParseCommand{
  private:
    RobotDataSDK state_rec; /**< Used to save received data.*/
    int is_data_recv = 0;
    const int local_port;
    void setDataRevState(int state){
      is_data_recv = state > 0? 1:0;
    }
    /// @brief CallBack_. 
    /// @param int Instruction type, only 0x0906.
    std::function<void(int)> CallBack_;
  public:
    /**
     * @brief  
     * @note   
     * @retval 
     */
    ParseCommand(int port=LOCAL_PORT);
   /**
    * @brief Create the accepted thread using the work()
    */
    void startWork(); /**< Create the accepted thread using the work().*/

   /**
    * @brief Receive robot data and save it
    */
    void work();
    /// @brief Registering Callbacks.
    void RegisterCallBack(std::function<void(int)> CallBack){
      CallBack_ = std::move(CallBack);
    }

    RobotDataSDK& getRecvState(); /**< Save the obtained data in st.*/

    void printData(); /**< Print the recieved data */

    int getDataRevState(){
      return is_data_recv;
    }
};
};
#endif  // PARSE_CMD_H_
