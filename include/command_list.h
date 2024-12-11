/**
 * @file cmmand.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef X30_COMMAND_LIST_H_
#define X30_COMMAND_LIST_H_

#include <deque>
#include "command.h"

namespace x30{
class CommandList {
  private:
    std::deque<Command*> command_list_;
    pthread_mutex_t mutex_;
    /// config list max size
    size_t list_capacity_;
  public:
    CommandList();
    virtual ~CommandList();

    uint32_t set_command(Command* command);
    uint32_t get_command_front(Command& command);
    uint32_t get_command_back(Command& command);
};
};//namespace x30
#endif  // COMMAND_H_