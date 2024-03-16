# -*- coding: UTF-8 -*-
import sys
sys.path.append('./lib')
import deeprobotics_motion_sdk_py as dr
import numpy as np

is_message_updated = False
kDegree2Radian = 3.1415926 / 180

goal_angle_fl = np.zeros(3) 
goal_angle_hl = np.zeros(3) 
goal_angle_fr = np.zeros(3)
goal_angle_hr = np.zeros(3)
init_angle_fl = np.zeros(3)
init_angle_fr = np.zeros(3)
init_angle_hl = np.zeros(3)
init_angle_hr = np.zeros(3)

class MotionExample:
    def __init__(self):
        self.init_time = 0.0
     
    def CubicSpline(self, init_pos, init_vel, goal_pos, goal_vel, 
                  run_time, cycle_time, total_time):
        d = init_pos
        c = init_vel
        a = (goal_vel * total_time - 2 * goal_pos + init_vel * total_time + 
            2 * init_pos) /pow(total_time, 3)
        b = (3 * goal_pos - goal_vel * total_time - 2 * init_vel * total_time - 
            3 * init_pos) / pow(total_time, 2)

        if run_time > total_time:
            run_time = total_time
        sub_goal_position = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d

        if run_time + cycle_time > total_time:
            run_time = total_time - cycle_time
        sub_goal_position_next = a * pow(run_time + cycle_time, 3) + \
                                b * pow(run_time + cycle_time, 2) + \
                                c * (run_time + cycle_time) + d

        if run_time + cycle_time * 2 > total_time:
            run_time = total_time - cycle_time * 2
        sub_goal_position_next2 = a * pow(run_time + cycle_time * 2, 3) + \
                                b * pow(run_time + cycle_time * 2, 2) + \
                                c * (run_time + cycle_time * 2) + d
        return sub_goal_position, sub_goal_position_next, sub_goal_position_next2
  
    def SwingToAngle(self, init_angle, final_angle, total_time, run_time, cycle_time, side,
                       cmd, data):
        goal_angle = np.zeros(3)
        goal_angle_next = np.zeros(3)
        goal_angle_next2 = np.zeros(3)
        goal_velocity = np.zeros(3)
        leg_side = 0

        if side == "FL":
            leg_side = 0
        elif side == "FR":
            leg_side = 1
        elif side == "HL":
            leg_side = 2
        elif side == "HR":
            leg_side = 3
        else:
            print("Leg Side Error!!!")

        # final_angle = final_angle
        for j in range(0, 3):
            goal_angle[j], goal_angle_next[j], goal_angle_next2[j] = \
                self.CubicSpline(init_angle[j], 0, final_angle[j], 0, run_time,
                                cycle_time, total_time)
            goal_velocity[j] = (goal_angle_next[j] - goal_angle[j]) / cycle_time


        cmd.joint_cmd[3 * leg_side].kp = 60
        cmd.joint_cmd[3 * leg_side + 1].kp = 60
        cmd.joint_cmd[3 * leg_side + 2].kp = 60
        cmd.joint_cmd[3 * leg_side].kd = 0.7
        cmd.joint_cmd[3 * leg_side + 1].kd = 0.7
        cmd.joint_cmd[3 * leg_side + 2].kd = 0.7
        cmd.joint_cmd[3 * leg_side].pos = goal_angle[0]
        cmd.joint_cmd[3 * leg_side + 1].pos = goal_angle[1]
        cmd.joint_cmd[3 * leg_side + 2].pos = goal_angle[2]
        cmd.joint_cmd[3 * leg_side].vel = goal_velocity[0]
        cmd.joint_cmd[3 * leg_side + 1].vel = goal_velocity[1]
        cmd.joint_cmd[3 * leg_side + 2].vel = goal_velocity[2]
        for i in range(0, 12):
            cmd.joint_cmd[i].tor = 0

    
    def PreStandUp(self, cmd, time, data_state):
        standup_time = 1.0
        cycle_time = 0.001
        goal_angle_fl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
        goal_angle_fr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
        goal_angle_hl = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]
        goal_angle_hr = [0 * kDegree2Radian, -70 * kDegree2Radian, 150 * kDegree2Radian]

        if time <= self.init_time + standup_time:
            self.SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                                cycle_time, "FL", cmd, data_state)
            self.SwingToAngle(init_angle_fl, goal_angle_fr, standup_time, time - self.init_time,
                                cycle_time, "FR", cmd, data_state)
            self.SwingToAngle(init_angle_fl, goal_angle_hl, standup_time, time - self.init_time,
                                cycle_time, "HL", cmd, data_state)
            self.SwingToAngle(init_angle_fl, goal_angle_hr, standup_time, time - self.init_time,
                                cycle_time, "HR", cmd, data_state)
   
    def StandUp(self, cmd, time, data_state):
        standup_time = 1.5
        cycle_time = 0.001
        goal_angle_fl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian] 
        goal_angle_fr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
        goal_angle_hl = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]
        goal_angle_hr = [0 * kDegree2Radian, -42 * kDegree2Radian, 78 * kDegree2Radian]

        if (time <= self.init_time + standup_time):
            self.SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - self.init_time,
                                cycle_time, "FL", cmd, data_state)
            self.SwingToAngle(init_angle_fr, goal_angle_fr, standup_time, time - self.init_time,
                                cycle_time, "FR", cmd, data_state)
            self.SwingToAngle(init_angle_hl, goal_angle_hl, standup_time, time - self.init_time,
                                cycle_time, "HL", cmd, data_state)
            self.SwingToAngle(init_angle_hr, goal_angle_hr, standup_time, time - self.init_time,
                                cycle_time, "HR", cmd, data_state)
        else:
            for i in range(0, 12):
                cmd.joint_cmd[i].tor = 0
                cmd.joint_cmd[i].kp = 80
                cmd.joint_cmd[i].kd = 0.7

        for i in range(0, 4):
            cmd.joint_cmd[3*i].pos = 0
            cmd.joint_cmd[3*i+1].pos = -42 * kDegree2Radian
            cmd.joint_cmd[3*i+2].pos = 78 * kDegree2Radian
            cmd.joint_cmd[3*i].vel = 0
            cmd.joint_cmd[3*i+1].vel = 0
            cmd.joint_cmd[3*i+2].vel = 0
        
    def GetInitData(self, data, time):
        self.init_time = time
        init_angle_fl[0] = data.joint_data[0].pos
        init_angle_fl[1] = data.joint_data[1].pos
        init_angle_fl[2] = data.joint_data[2].pos

        init_angle_fr[0] = data.joint_data[3].pos
        init_angle_fr[1] = data.joint_data[4].pos
        init_angle_fr[2] = data.joint_data[5].pos

        init_angle_hl[0] = data.joint_data[6].pos
        init_angle_hl[1] = data.joint_data[7].pos
        init_angle_hl[2] = data.joint_data[8].pos

        init_angle_hr[0] = data.joint_data[9].pos
        init_angle_hr[1] = data.joint_data[10].pos
        init_angle_hr[2] = data.joint_data[11].pos

def handler(code):
    if(code == 0x0906):
        is_message_updated = True
    

if __name__ == "__main__" :
    timer = dr.TimeTool()
    now_time = 0.0
    start_time = 0.0

    sender = dr.SendToRobot('192.168.1.103', 43893)
    receiver = dr.ParseCommand(43897)
    # receiver.register_call_back(handler)
    robot_data = receiver.get_state()
    receiver.start_work()
    timer.time_init(1)
    sender.robot_state_init()
    
    start_time = timer.get_start_time()
    
    robot_set_up_demo = MotionExample()
    robot_set_up_demo.GetInitData(receiver.get_state(), 0.000)
    
    time_tick = 0
    robot_joint_cmd = dr.RobotCmdSDK()
  
    while True:
        if timer.time_interrupt() == True:
          continue
        now_time = timer.get_now_time(start_time)
        time_tick = time_tick+1

        if time_tick < 1000:
            robot_set_up_demo.PreStandUp(robot_joint_cmd, now_time, receiver.get_state())
        if time_tick == 1000:
            robot_set_up_demo.GetInitData(receiver.get_state(), now_time)
        if time_tick >= 1000:
            robot_set_up_demo.StandUp(robot_joint_cmd, now_time, receiver.get_state())
        if time_tick >= 10000:
            sender.control_get(1)
            break
        if is_message_updated:
            pass
        sender.set_send(robot_joint_cmd)
