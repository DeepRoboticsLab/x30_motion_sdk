import sys
sys.path.append('./lib')
import deeprobotics_motion_sdk_py as dr
import numpy as np
import threading 

class RobotState:
    def __init__(self) -> None:
        self.rpy        = np.zeros((3, 1))
        self.omega_body = np.zeros((3, 1)) 
        self.joint_pos  = np.zeros((12, 1))
        self.joint_vel  = np.zeros((12, 1))
    

class NerualNetworkController():
    def __init__(self) -> None:
        self.robot_state = RobotState()
        
        self.receiver = dr.ParseCommand(43897)
        self.receiver.start_work()
        # pass
    
    def parse_robot_state(self):
        robot_data = self.receiver.get_state()
        self.robot_state.rpy = np.array([[robot_data.imu.roll], [robot_data.imu.pitch], [robot_data.imu.yaw]])
        self.robot_state.omega_body = np.array([[robot_data.imu.omega_x], [robot_data.imu.omega_y], [robot_data.imu.omega_z]])
        for i in range(12):
            self.robot_state.joint_pos[i] = robot_data.joint_data[i].pos
            self.robot_state.joint_vel[i] = robot_data.joint_data[i].vel
            
    def run(self):
        pass
            
            
if __name__ == "__main__" :
    nnc = NerualNetworkController()
    nnc.parse_robot_state()