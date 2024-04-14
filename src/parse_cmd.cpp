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
using namespace x30;

ParseCommand::ParseCommand(int port):local_port(port){
  memset(&state_rec, 0, sizeof(state_rec));
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
          if(CallBack_){
            CallBack_(STATE_RECEIVE_CODE);
          }
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

void ParseCommand::printData(){
  std::cout.precision(3);
  std::cout << "time_tick: " << state_rec.tick << std::endl;
  std::cout << "Imu Data:  " << std::endl;
  std::cout << "rpy:  " << state_rec.imu.roll << " " << state_rec.imu.pitch << " " << state_rec.imu.yaw << std::endl;
  std::cout << "gyro: " << state_rec.imu.omega_x << " " << state_rec.imu.omega_y << " " << state_rec.imu.omega_z << std::endl;
  std::cout << "acc:  " << state_rec.imu.acc_x << " " << state_rec.imu.acc_y << " " << state_rec.imu.acc_z << std::endl;

  std::cout << "Leg Data:  " << std::endl;
  std::cout << "pos:  ";
  for(int i=0;i<12;++i) std::cout << state_rec.joint_data[i].pos << " ";
  std::cout << std::endl;
  std::cout << "vel:  ";
  for(int i=0;i<12;++i) std::cout << state_rec.joint_data[i].vel << " ";
  std::cout << std::endl;
  std::cout << "tor:  ";
  for(int i=0;i<12;++i) std::cout << state_rec.joint_data[i].tor << " ";
  std::cout << std::endl;
}
