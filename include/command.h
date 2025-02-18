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

#ifndef X30_COMMAND_H_
#define X30_COMMAND_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <bitset>

namespace x30{
struct EthCommand{
  uint32_t code;
  union{
    uint32_t value;
    uint32_t paramters_size;
  };
  struct {
    /// indicate the massage whether has one more
    /// @param 1 multiple value;
    /// @param 0 single value;
    uint32_t type  : 8;
    /// the sequence number of massage
    uint32_t count : 24;
  };
};

namespace command_type{
  enum CommandType{
    kSingleValue = 0,
    kMessValues = 1,
  };
}

const size_t kCommandDataBufferSize = 1024;

struct CommandMessage{
  EthCommand command;
  char data_buffer[kCommandDataBufferSize];
};

class Command {
  private:
    std::bitset<32> command_code_;
    union{
      int32_t command_value_;
      size_t command_parameters_size_;
    };
    void* command_parameters_;

  public:
    Command();
    Command(uint32_t command_code,int32_t command_value);
    Command(uint32_t command_code,size_t command_parameters_size,void* command_parameters);

    virtual ~Command();

    std::bitset<32>& get_command_code();
    int32_t get_command_value();
    size_t get_command_parameters_size();
    void* get_command_parameters();

    friend std::ostream& operator<<(std::ostream& stream, Command& c);
}; // end of Command

};//namespace x30
#endif  // COMMAND_H_