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

#include "parse_cmd.h"
#include "command_list.h"
using namespace std;

ParseCommand::ParseCommand(int port):local_port(port){

}

void ParseCommand::work()
{
  CommandMessage cm;
  UDPServer udpServer;
  timespec test_time;

  udpServer.onRawMessageReceived = [&](const char* message, int length, string ipv4, uint16_t port) {
    clock_gettime(1, &test_time);
    memcpy(&cm,message,sizeof(cm));
    Command nc(cm.command.code,cm.command.paramters_size, cm.data_buffer);
    if(cm.command.type == command_type::CommandType::kMessValues){
      switch (cm.command.code){
        case STATE_RECEIVE_CODE:
          clock_gettime(1, &test_time);
          memcpy(&state_rec, cm.data_buffer, sizeof(state_rec));
          break;
      default:
        break;
      }
    }
    if(message==nullptr||length==0){
      setDataRevState(0);
    }else{
      setDataRevState(1);
    }
  };
    // Bind the server to a port.
    udpServer.Bind(local_port, [](int errorCode, string errorMessage) {
    // BINDING FAILED:
    cout << errorCode << " : " << errorMessage << endl;
  });


	while (1){
    sleep(1);
	}
}

void ParseCommand::startWork()
{
  std::thread work_thread(std::bind(&ParseCommand::work, this));
	work_thread.detach();
}
RobotDataSDK& ParseCommand::getRecvState(){
  return state_rec;
}
