/**
 * @file motion_sdk_pybind.cpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#include "parse_cmd.h"
#include "send_to_robot.h"
#include "x30_types.h"
#include <functional>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/embed.h>
#include <pybind11/pytypes.h>
#include <pybind11/numpy.h>
#include <array>

using namespace x30;
namespace py = pybind11;


PYBIND11_MODULE(deeprobotics_x30_motion_sdk_py, m) {
  py::class_<ImuDataSDK>(m, "ImuDataSDK")
    .def(py::init<>())
    .def_readwrite("timestamp", &ImuDataSDK::timestamp)
    .def_readwrite("roll", &ImuDataSDK::roll)
    .def_readwrite("pitch", &ImuDataSDK::pitch)
    .def_readwrite("yaw", &ImuDataSDK::yaw)
    .def_readwrite("omega_x", &ImuDataSDK::omega_x)
    .def_readwrite("omega_y", &ImuDataSDK::omega_y)
    .def_readwrite("omega_z", &ImuDataSDK::omega_z)
    .def_readwrite("acc_x", &ImuDataSDK::acc_x)
    .def_readwrite("acc_y", &ImuDataSDK::acc_y)
    .def_readwrite("acc_z", &ImuDataSDK::acc_z);

  py::class_<SingleJointData>(m, "SingleJointData")
    .def(py::init<>())
    .def_readwrite("pos", &SingleJointData::pos)
    .def_readwrite("vel", &SingleJointData::vel)
    .def_readwrite("tor", &SingleJointData::tor)
    .def_readwrite("temperature", &SingleJointData::temperature);

  py::class_<SingleJointCmd>(m, "SingleJointCmd")
    .def(py::init<>())
    .def_readwrite("pos", &SingleJointCmd::pos)
    .def_readwrite("vel", &SingleJointCmd::vel)
    .def_readwrite("tor", &SingleJointCmd::tor)
    .def_readwrite("kp", &SingleJointCmd::kp)
    .def_readwrite("kd", &SingleJointCmd::kd);

  py::class_<RobotCmdSDK>(m, "RobotCmdSDK")
    .def(py::init<>())
    .def_readwrite("joint_cmd", &RobotCmdSDK::joint_cmd);
  
  py::class_<RobotDataSDK>(m, "RobotDataSDK")
    .def(py::init<>())
    .def_readwrite("tick", &RobotDataSDK::tick)
    .def_readwrite("imu", &RobotDataSDK::imu)
    .def_readwrite("joint_data", &RobotDataSDK::joint_data)
    .def_readwrite("contact_force", &RobotDataSDK::contact_force);
  

  py::class_<TimeTool>(m, "TimeTool")
    .def(py::init<>())
    .def("time_init", &TimeTool::time_init)
    .def("time_interrupt", &TimeTool::time_interrupt)
    .def("get_now_time", &TimeTool::get_now_time)
    .def("get_start_time", &TimeTool::get_start_time);

  py::class_<ParseCommand>(m, "ParseCommand")
    .def(py::init<int32_t>())
    .def("register_call_back", &ParseCommand::RegisterCallBack)
    .def("start_work", &ParseCommand::startWork)
    .def("get_data", &ParseCommand::getRecvState)
    .def("get_receive_state", &ParseCommand::getDataRevState)
    .def("print_data", &ParseCommand::printData);;

  py::class_<SendToRobot>(m, "SendToRobot")
    .def(py::init<std::string , uint16_t>())
    .def("set_send", &SendToRobot::set_send)
    .def("control_get", &SendToRobot::control_get)
    .def("robot_state_init", &SendToRobot::robot_state_init);
}
