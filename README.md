# 绝影X30运动控制SDK


&nbsp;
##  1 远程连接

开发者可通过SSH远程连接到运动主机。

- 将开发主机连接到机器人WiFi。

- 在开发主机上打开SSH连接软件，输入`ssh ysc@192.168.1.103`，密码为 `'` [英文单引号]，即可远程连接运动主机。

- 输入以下命令以打开网络配置文件：
	```Bash
	cd /home/ysc/jy_exe/conf/
	vim network.toml
	```
- 配置文件 ***network.toml*** 内容如下：
	```toml
	ip = '192.168.1.103'
	target_port = 43897
	local_port = 43893
	ips = ["192.168.1.105","192.168.1.106","192.168.1.xxx"]
	ports = [43897,43897,43897]
	```
	
- 修改配置文件第4行`ips`中的第3个IP地址，使得 **MotionSDK** 能够接收到机器狗数据:
	- 如果 **MotionSDK** 在机器人运动主机内运行，IP设置为运动主机IP：`192.168.1.103`；  
	- 如果 **MotionSDK** 在开发者自己的开发主机中运行，设置为开发主机的静态IP：`192.168.1.xxx`。
	
- 重启运动程序使配置生效：
	```bash
	cd /home/ysc/jy_exe
	sudo ./stop.sh
	sudo ./restart.sh
	```


&nbsp;
##  2 编译开发

- 编译开发时，开发者可进入解压得到的文件夹，在***CMakeLists.txt*** 的同级目录下新建 ***build*** 文件夹；

	```bash
	cd xxxxxxxx     # cd <path to where you want to create build directory>
	mkdir build
	```
	
	> 注意：开发者可在任何地方创建 ***build*** 文件夹，但在编译时，`cmake` 指令须指向 ***CMakeLists.txt*** 所在的路径。

- 打开 ***build*** 文件夹并编译；

   ```bash
	cd build
	cmake .. 
	make -j
	```
	
- 编译结束后，会在 ***build*** 目录下生成一个名为 ***X30_motion*** 的可执行文件，此即为我们代码编译出来的结果；

- 在终端中继续输入以下命令行以运行程序：

	```bash
	./X30_motion
	```
**在X30_MotionSDK的main.cpp中，倒数几行有一行被注释的指令下发代码**：

```c++
//send2robot_cmd->set_send(robot_joint_cmd);
```

**此为SDK下发指令的调用，为了确保SDK的安全使用，这行下发指令默认是注释掉的，机器狗默认只会回零不会起立。**

&nbsp;
##  3 示例代码

本节对 ***main.cpp*** 进行说明。  

定时器，用于设置算法周期，获得当前时间：

```cpp
TimeTool my_set_timer;
my_set_timer.time_init(int);                              		  ///< Timer initialization, input: cycle; unit: ms
my_set_timer.get_start_time();                           		  ///< Obtain time for algorithm
my_set_timer.time_interrupt()			      		              ///< Timer interrupt flag
my_set_timer.get_now_time(double);               		          ///< Get the current time
```

SDK在绑定机器人的IP和端口后，获取控制权， 发送关节控制指令：

```cpp
SendToRobot* send2robot_cmd = new SendToRobot("192.168.1.103",43893);   ///< Create a sender thread
send2robot_cmd->robot_state_init();                           		    ///< Reset all joints to zero and gain control right
send2robot_cmd->set_send(RobotCmdSDK); 			     		            ///< Send joint control command
send2robot_cmd->control_get(int);                            		    ///< Return the control right
```

SDK接收机器人下发的关节数据：

```cpp
ParseCommand* robot_data_rec = new ParseCommand;           		  ///< Create a thread for receiving and parsing
robot_data_rec->getRecvState(); 			      		          ///< Receive data from 12 joints

```

SDK接收到的关节数据将保存在`robot_data`中：

```cpp
RobotDataSDK *robot_data = &robot_data_rec->getRecvState(); 		  ///< Saving joint data to the robot_data
///< Left front leg：fl_leg[3], the sequence is fl_hipx, fl_Hipy, fl_knee
///< Right front leg：fr_leg[3], the sequence is fr_hipx, fr_Hipy, fr_knee
///< Left hind leg：hl_leg[3], the sequence is hl_hipx, hl_Hipy, hl_knee
///< Right hind leg：hr_leg[3], the sequence is hr_hipx, hr_Hipy, hr_knee
///< All joints：leg_force[12]/joint_data[12], the sequence is fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_Hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee
	
robot_data->fl_force[]				  ///< Contact force on left front foot in X-axis, Y-axis and Z-axis
robot_data->fr_force[]				  ///< Contact force on right front foot in X-axis, Y-axis and Z-axis
robot_data->hl_force[]				  ///< Contact force on left hind foot in X-axis, Y-axis and Z-axis
robot_data->hr_force[]				  ///< Contact force on right hind foot in X-axis, Y-axis and Z-axis
robot_data->contact_force[]			  ///< Contact force on all feet
	
robot_data->tick						  ///< Cycle of operation
	
robot_data->imu							      ///< IMU data	
robot_data->imu.acc_x						  ///< Acceleration on X-axis, unit m/s^2
robot_data->imu.acc_y						  ///< Acceleration on Y-axis, unit m/s^2
robot_data->imu.acc_z						  ///< Acceleration on Z-axis, unit m/s^2
robot_data->imu.roll					      ///< Roll angle, unit deg
robot_data->imu.pitch					      ///< Pitch angle, unit deg
robot_data->imu.yaw					          ///< Yaw angle, unit deg
robot_data->imu.omega_x			  	          ///< angular velocity on X-axis, unit rad/s
robot_data->imu.omega_y			  	          ///< angular velocity on Y-axis, unit rad/s
robot_data->imu.omega_z		   	 	          ///< angular velocity on Z-axis, unit rad/s
robot_data->imu.buffer_byte					  ///< Buffer data
robot_data->imu.buffer_float			      ///< Buffer data
robot_data->imu.timestamp					  ///< Time when the data is obtained

robot_data->joint_state						  ///< Motor status
robot_data->joint_state.fl_leg[].position	  ///< Motor position of left front leg
robot_data->joint_state.fl_leg[].temperature  ///< Motor temperature of left front leg
robot_data->joint_state.fl_leg[].torque		  ///< Motor torque of left front leg 
robot_data->joint_state.fl_leg[].velocity	  ///< Motor velocity of left front leg
robot_data->joint_state.joint_data            ///< All joint data
```

机器人关节控制指令：

```cpp
RobotCmdSDK robot_joint_cmd;  					  ///< Target data of each joint
///< Left front leg：fl_leg[3], the sequence is fl_hipx, fl_Hipy, fl_knee
///< Right front leg：fr_leg[3], the sequence is fr_hipx, fr_Hipy, fr_knee
///< Left hind leg：hl_leg[3], the sequence is hl_hipx, hl_Hipy, hl_knee
///< Right hind leg：hr_leg[3], the sequence is hr_hipx, hr_Hipy, hr_knee
///< All joints：leg_force[12]/joint_data[12], the sequence is fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_Hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee

robot_joint_cmd.fl_leg[]->kd;					  ///< Kd of left front leg
robot_joint_cmd.fl_leg[]->kp;					  ///< Kp of left front leg
robot_joint_cmd.fl_leg[]->position;				  ///< Position of left front leg
robot_joint_cmd.fl_leg[]->torque;				  ///< Torue of left front leg
robot_joint_cmd.fl_leg[]->velocity;				  ///< Velocity of left front leg
```

机器人站立的简单demo：  
1.将机器人腿收起来，为站立做准备；  
2.记录下当前时间与关节数据；  
3.机器人起立。

```cpp
MotionExample robot_set_up_demo;                      		  ///< Demo for testing

/// @brief Spend 1 sec drawing the robot's legs in and preparing to stand
/// @param cmd Send control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*robot_data);	

/// @brief Only the current time and angle are recorded
/// @param data Current joint data
/// @param time Current timestamp
robot_set_up_demo.GetInitData(robot_data->motor_state,now_time);	

/// @brief Spend 1.5 secs standing up
/// @param cmd Send control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*robot_data);
```


&nbsp;
##  4 常见问题与注意事项
### 问题一

**问：** 在自己的开发主机运行MotionSDK时，如何判断SDK是否与机器狗正常通讯？

**答：**

SDK采用UDP与机器狗进行通讯。

针对数据上报，可以在SDK里打印关节数据或陀螺仪数据等信息，以此判断是否收到机器狗上报的SDK数据。或者观察SDK运行时，是否打印connection refused，以此判断是否收到机器狗上报的SDK数据。

针对指令下发，如果SDK运行后，机器狗做出回零动作，则证明SDK能成功下发指令到机器狗本体。

### 问题二

**问：** 如果SDK没收到机器狗上报数据，如何解决？

**答：**

首先检查开发主机是否与机器狗主机能处于同一网段下，如果是在机器狗上运行SDK，此步骤可跳过。

开发者先连接机器狗的WiFi网络，然后在自己的开发主机上`ping 192.168.1.103`，ping通之后ssh连接到机器狗运动主机内，在运动主机内`ping 192.168.1.xxx`，xxx为开发者开发主机的静态ip。

如果上述步骤失败，可能需要开发者手动设置自己开发主机的ip地址。

如果开发者的开发环境为虚拟机，建议把虚拟机网络连接方式改为桥接并手动设置虚拟机ip地址后重启虚拟机。

其次检查是否按照教程里的"远程连接"部分正确设置机器狗上的配置文件。

如果仍收不到机器狗上报数据，可在机器狗运动主机上运行`sudo tcpdump -x port 43897`，等待2分钟，观察机器狗是否有原始数据上报。如果没有，输入top命令查看机器狗本体控制程序进程jy_exe是否正常运行，若jy_exe没有正常运行，参照以下指令重启运动程序：

```bash
 cd /home/ysc/jy_exe
 sudo ./stop.sh
 sudo ./restart.sh
```



### 问题三

**问：** 下发控制指令不生效，机器人没反应？

**答：**

在X30_MotionSDK的main.cpp中，倒数几行有一行被注释的指令下发代码：

```c++
//send_cmd->SendCmd(robot_joint_cmd); 
```

此为SDK下发指令的调用，为了确保SDK的安全使用，这行**下发指令默认是注释掉的，机器狗默认只会回零不会起立。**

取消注释前，请开发者务必确认上述问题一、二，确保SDK与机器狗正常通讯，同时确保自己的下发控制指令正确，否则机器狗执行控制指令时可能会产生危险！



### 问题四

**问：** 控制指令是如何生效的？

**答：**

提供goal_pos,goal_vel,kp,kd,t_ff，共5个关节控制参数接口，控制接口全为低速端，也就是**关节端**，最终关节目标力为
$$
T=kp*(pos_{goal} - pos_{real})+kd*(vel_{goal} - vel_{real})+t_{ff}
$$
驱动器端会将最终的关节目标力转化成期望电流，并以20kHz的频率进行闭环控制。

**使用举例：**

当做纯位控即位置控制时，电机的输出轴将会稳定在一个固定的位置。例如，如果我们希望电机输出端固定在3.14弧度的位置，下发数据格式示例：
$$
pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=0, t_{ff} = 0
$$
当做速度控制时，下发数据格式示例：
$$
pos_{goal}=0, vel_{goal}=5, kp=0, kd=1, t_{ff} = 0
$$
当做阻尼控制时，下发数据格式示例：
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=1, t_{ff} = 0
$$
当做力矩控制时，下发数据格式示例：
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 3
$$
当做零力矩控制时，下发数据格式示例：
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 0
$$
当做混合控制时，下发数据格式示例：
$$
pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=1, t_{ff} = 1
$$


### 问题五

**问：** SDK的控制逻辑是怎样的？

**答：**

当SDK有指令下发时，底层控制器会优先执行SDK的控制指令，并把指令分发给机器狗12个关节。当SDK没有指令下发时，经过1s的超时判断后，底层控制器会拿回控制权，进入阻尼保护模式一段时间后，清空关节指令。控制流程图可以参考下图：
<img src="./doc/MotionControlFlow.png" alt="a" style="zoom:100%;" />




### 问题六 

**问：** 如何修改关节力矩限幅？

**答：**
在机器狗运动主机内，执行如下指令
```bash
 cd /home/ysc/jy_exe/conf
 vim sdk_config.toml
```
你将看到如下内容

```toml
torque_limit=[42.0,42.0,90.0]   
```
该数组的第1，2，3个数分别代表髋关节(hipx)，侧摆关节(hipy)，膝关节(knee)的力矩限幅，你可按需修改，建议在刚开始测试程序的时候将力矩限幅设置的较小一些，待程序验证无误后再放开力矩限幅，确保机器人处于安全的状态。

**修改后，执行如下指令重启运动程序，使得修改生效**，
```bash
cd /home/ysc/jy_exe
sudo ./stop.sh
sudo ./restart.sh
```



### 其他注意事项

1. X30运动主机是ARM架构的，如果开发者想在运动主机上编译自己的程序，需要注意。
2. WiFi通讯受网络环境干扰产生的通讯延迟波动，可能对控制频率在500Hz以上的控制器有一定的影响。



## 5 硬件参数

### 1.身体与关节坐标系

<img src="./doc/body.png" alt="a" style="zoom:100%;" />

<img src="./doc/leg.png" alt="a" style="zoom:100%;" />

【注意】弧形箭头指示颜色相同的关节坐标系的旋转正方向。



### 2.身体连杆参数

<img src="./doc/body_length.png" alt="a" style="zoom:100%;" />

| 参数                 | 数值     | 说明                 |
| -------------------- | -------- | -------------------- |
| 长度(Lbody)          | 0.98m    | 身体总长度           |
| 髋前后间距(Lhip)     | 0.584m   | 前后髋关节中心距离   |
| 髋左右间距(Whip)     | 0.16m    | 左右髋关节中心的距离 |
| 腿平面左右间距(Wleg) | 0.39284m | 腿平面左右的距离     |
| 宽度(Wbody)          | 0.47m    | 身体总宽度           |



### 3.腿部连杆参数

<img src="./doc/leg_length.png" alt="s" style="zoom:75%;" />

| 参数                       | 数值     | 说明                           |
| -------------------------- | -------- | ------------------------------ |
| 腿平面与髋侧摆关节距离(L1) | 0.11642m | 髋侧摆关节与腿平面距离         |
| 大腿长度(L2)               | 0.3m     | 髋前摆关节中心与膝关节中心距离 |
| 小腿长度(L3)               | 0.31m    | 膝关节中心与足底圆心距离       |
| 足底半径                   | 0.03m    | 足底缓冲件半斤                 |



### 4.关节参数

| **关节**     | **运动范围** | **额定转矩** | **额定转速** | **峰值转矩** |
| ------------ | ------------ | ------------ | ------------ | ------------ |
| 髋侧摆(HipX) | -18.5°~33.5° | 28Nm         | 12rad/s      | 84Nm         |
| 髋前摆(HipY) | -170°~15°    | 28Nm         | 12rad/s      | 84Nm         |
| 膝关节(Knee) | 20°~145°     | 65Nm         | 11rad/s      | 180Nm        |



【注意】其他有关于X30四足机器人的动力学参数可以在提供的URDF文件中获得
