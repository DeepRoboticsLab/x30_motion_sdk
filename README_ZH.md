# 绝影X30运动控制SDK

[English](./README.md)

&nbsp;
## 1 SDK更新记录

### V1.0（2024-01-20） 
**[新增]** 首次发布。 

&nbsp;
##  2 SDK简介
**MotionSDK** 提供了低速端（即**关节端**）的5个控制参数接口： $pos_{goal}$， $vel_{goal}$， $kp$， $kd$， $t_{ff}$。

当SDK有指令下发时，底层控制器会优先执行SDK的控制指令，将指令分发给机器狗的12个关节。底层关节根据5个控制参数可计算出最终关节目标力为：
$$T=kp*(pos_{goal} - pos_{real})+kd*(vel_{goal} - vel_{real})+t_{ff}$$

驱动器端会将最终的关节目标力转化成期望电流，并以20kHz的频率进行闭环控制。

当SDK没有指令下发超过1s时，底层控制器会拿回控制权，进入阻尼保护模式一段时间后，清空关节指令。

**控制流程可以参考下图：**

<img src="./doc/MotionControlFlow.png" alt="a" style="zoom:100%;" />

**控制参数使用举例：**

当做纯位控即位置控制时，电机的输出轴将会稳定在一个固定的位置。例如，如果我们希望电机输出端固定在3.14弧度的位置，下发数据格式示例：
$$pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=0, t_{ff} = 0$$
当做速度控制时，下发数据格式示例：
$$pos_{goal}=0, vel_{goal}=5, kp=0, kd=1, t_{ff} = 0$$
当做阻尼控制时，下发数据格式示例：
$$pos_{goal}=0, vel_{goal}=0, kp=0, kd=1, t_{ff} = 0$$
当做力矩控制时，下发数据格式示例：
$$pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 3$$
当做零力矩控制时，下发数据格式示例：
$$pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 0$$
当做混合控制时，下发数据格式示例：
$$pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=1, t_{ff} = 1$$

&nbsp;
## 3 硬件参数

### 3.1 身体与关节坐标系

<img src="./doc/body.png" alt="a" style="zoom:100%;" />

<img src="./doc/leg.png" alt="a" style="zoom:100%;" />

> 注意：弧形箭头指示与其颜色一致的关节坐标系的旋转正方向。

### 3.2 身体连杆参数

<img src="./doc/body_length.png" alt="a" style="zoom:100%;" />

| 参数                 | 数值     | 说明                 |
| -------------------- | -------- | -------------------- |
| 长度(Lbody)          | 0.98m    | 身体总长度           |
| 髋前后间距(Lhip)     | 0.582m   | 前后髋关节中心距离   |
| 髋左右间距(Whip)     | 0.16m    | 左右髋关节中心的距离 |
| 腿平面左右间距(Wleg) | 0.3935m | 腿平面左右的距离     |
| 宽度(Wbody)          | 0.454m    | 身体总宽度           |

### 3.3 腿部连杆参数

<img src="./doc/leg_length.png" alt="s" style="zoom:75%;" />

| 参数                       | 数值     | 说明                           |
| -------------------------- | -------- | ------------------------------ |
| 腿平面与髋侧摆关节距离(L1) | 0.1167m | 髋侧摆关节与腿平面距离         |
| 大腿长度(L2)               | 0.3m     | 髋前摆关节中心与膝关节中心距离 |
| 小腿长度(L3)               | 0.31m    | 膝关节中心与足底圆心距离       |
| 足底半径                   | 0.039m    | 足底缓冲件半径                 |

### 3.4 关节参数

| **关节**     | **运动范围** | **额定转矩** | **额定转速** | **峰值转矩** |
| ------------ | ------------ | ------------ | ------------ | ------------ |
| 髋侧摆(HipX) | -18.5°~33.5° | 28Nm         | 12rad/s      | 84Nm         |
| 髋前摆(HipY) | -170°~15°    | 28Nm         | 12rad/s      | 84Nm         |
| 膝关节(Knee) | 24°~140°     | 65Nm         | 11rad/s      | 180Nm        |

> 注意：绝影X30四足机器人的其他动力学参数可以在提供的URDF文件中获得。

&nbsp;
## 4 SDK下载及解压

- 下载 **X30_MotionSDK**，并解压。

&nbsp; 
## 5 配置SDK参数和数据上报地址

开发者可通过ssh远程连接到运动主机，以配置运动主机上报关节等运动数据的目标地址和sdk相关参数。

- 将开发主机连接到机器狗WiFi。

- 在开发主机上打开ssh连接软件，输入`ssh ysc@192.168.1.103`，密码为 `'` [英文单引号]，即可远程连接运动主机。

- 输入以下命令以打开网络配置文件：
	```Bash
	cd ~/jy_exe/conf/
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
	
- 运动程序运行时，会向配置文件中的`ip`和`ips`所包含的地址上报运动状态数据。

	- 如果 **MotionSDK** 在机器狗运动主机内运行，请查看`ip`或`ips`是否已包含运动主机IP `192.168.1.103`，如果未包含，请在`ips`中添加，并在`ports`中对应的位置也添加上接受数据的端口号，默认是`43897`；

	- 如果 **MotionSDK** 在开发者自己的开发主机中运行，请在`ips`中添加开发主机的静态IP `192.168.1.xxx`，并在`ports`中对应的位置添加上接受数据的端口号，默认是`43897`。

- 数据上报地址配置完成后，需要将运动主机向sdk上报数据的开关打开，首先打开配置文件 ***sdk_config.toml*** :
	```Bash
	cd ~/jy_exe/conf/
	vim sdk_config.toml
	```

- 修改配置文件 ***sdk_config.toml*** 中`enable_joint_data`的值：
	```toml
	enable_joint_data = true
	```
- 在同一个配置文件 ***sdk_config.toml*** 中，可按需对关节力矩限幅进行修改：
	```toml
	torque_limit=[42.0,42.0,90.0]
	```
	数组元素定义如下：

	| 数组元素 | 含义     | 取值范围  |数据类型|
	| --- | -------- | ---------- | ---------- |
	| torque_limit[0] | 对髋关节hipx的力矩限幅(N·m)| (0,84.0] | double |
	| torque_limit[1] | 对侧摆关节hipy的力矩限幅(N·m)| (0,84.0] | double |
	| torque_limit[2] | 对膝关节knee的力矩限幅(N·m)| (0,160.0] | double |

	> 注意：在刚开始测试一个新的运动程序的时候应将力矩限幅设置得较小一些，待程序验证无误后再放开力矩限幅，确保机器狗处于安全状态。

- 重启运动程序使配置生效：
	```bash
	cd ~/jy_exe
	sudo ./stop.sh
	sudo ./restart.sh
	```

&nbsp;
## 6 编译开发

***main.cpp***中提供了机器狗站立的简单demo，并在完成站立一段时间后将控制权归还给底层控制器，进入阻尼保护模式：

<img src="./doc/demoFlow.png" alt="a" style="zoom:100%;" />


**但为了确保SDK的安全使用，在*main.cpp*的原始代码中，第80行的下发指令代码是被注释掉的，因此机器狗默认只会调整到准备起立姿势但不会起立：**

```c++
//  send2robot_cmd->set_send(robot_joint_cmd);
```

> 注意：在取消注释前，开发者务必确保SDK与机器狗正常通讯（可参考“6.1 检查通讯”），并确保自己的下发控制指令正确，否则机器狗执行控制指令时可能会产生危险！

### 6.1 检查通讯

MotionSDK采用UDP与机器狗进行通讯。

针对指令下发，如果SDK运行后，机器狗做出准备起立姿势，则证明SDK能成功下发指令到机器狗本体。

针对数据上报，可以在SDK里打印关节数据或陀螺仪数据等信息，以此判断是否收到机器狗上报的SDK数据；或者观察SDK运行时，是否打印“No data from the robot was received!!!!!!”，以此判断是否收到机器狗上报的SDK数据。

- 首先对未取消下发指令注释的代码进行编译。

- 进入解压得到的文件夹，在***CMakeLists.txt*** 的同级目录下新建 ***build*** 文件夹；

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

- 在终端中继续输入以下命令行以运行程序（运行前请确保开发主机已连入机器狗网络）：

	```bash
	./X30_motion
	```

- 观察程序运行过程中机器狗是否做出准备起立姿势，以及终端中打印机器狗上报数据是否正常。

### 6.2 通讯问题排查

如果SDK未接收到机器狗上报的数据，可按照下述步骤进行排查：

- 首先检查开发主机是否与机器狗主机处于同一网段下（如果是在机器狗上运行SDK，此步骤可跳过）：将开发主机连到接机器狗的WiFi网络或尾部网口，然后在开发主机上ping运动主机，ping通之后ssh连接到机器狗运动主机内，在运动主机内ping开发主机的静态IP。如果无法ping通，请尝试手动设置自己开发主机的IP地址，并再次按照第5节对配置文件进行修改。

- 如果开发者的开发环境为虚拟机，建议把虚拟机网络连接方式改为桥接并手动设置虚拟机IP地址后重启虚拟机，并再次按照第5节对配置文件进行修改。

如果仍收不到机器狗上报数据，可在机器狗运动主机上抓包：

- 如果 **MotionSDK** 在机器狗运动主机内运行，运行`sudo tcpdump -x port 43897 -i lo`;

- 如果 **MotionSDK** 在开发者的开发主机内运行，运行`sudo tcpdump -x port 43897 -i eth1`。

执行抓包命令后等待2分钟，观察机器狗是否有原始数据上报。如果没有，输入top命令查看机器狗本体控制程序进程jy_exe是否正常运行，若jy_exe没有正常运行，参照以下指令重启运动程序：

```bash
 cd ~/jy_exe
 sudo ./stop.sh
 sudo ./restart.sh
```
### 6.3 编译开发

确保SDK与机器狗正常通讯，并确保自己的下发控制指令正确后，可以将***main.cpp***原始代码中第80行的下发指令代码`send2robot_cmd->set_send(robot_joint_cmd)`取消注释，然后重新编译运行:

- 删除之前生成的构建文件夹***build***：

- 打开一个新的终端，新建一个空的 ***build*** 文件夹；

	```bash
	cd xxxxxxxx     # cd <path to where you want to create build directory>
	mkdir build
	```
	
- 打开 ***build*** 文件夹并编译；
	```bash
	cd build

 	cmake .. -DBUILD_EXAMPLE=ON
 	make -j
 	```
- 编译结束后，会在 ***build/example*** 目录下生成一个名为 ***motion_example*** 的可执行文件，运行该文件时，机器狗将会执行下发的控制指令：

	```bash
	./example/motion_example
	```
> **注意：用户在使用X30执行算法和实验的过程中，请与机器狗保持至少5米距离，并将机器狗悬挂在调试架上避免意外造成人员和设备损伤。若实验过程中，机器狗摔倒或者用户想搬动机器狗位置，需要靠近机器狗时，用户应当使得机器狗处于急停状态或者使用 `sudo ./stop.sh` 命令关闭运动程序。**

&nbsp;
##  7 示例代码

本节对 ***main.cpp*** 进行说明。  

定时器，用于设置算法周期，获得当前时间：

```cpp
TimeTool my_set_timer;
my_set_timer.time_init(int);                              		  ///< Timer initialization, input: cycle; unit: ms
my_set_timer.get_start_time();                           		  ///< Obtain time for algorithm
my_set_timer.time_interrupt()			      		              ///< Timer interrupt flag
my_set_timer.get_now_time(double);               		          ///< Get the current time
```

SDK在绑定机器狗的IP和端口后，获取控制权，发送关节控制指令：

```cpp
SendToRobot* send2robot_cmd = new SendToRobot("192.168.1.103",43893);   ///< Create a sender thread
send2robot_cmd->robot_state_init();                           		    ///< Reset all joints to zero and gain control right
send2robot_cmd->set_send(RobotCmdSDK); 			     		            ///< Send joint control command
send2robot_cmd->control_get(int);                            		    ///< Return the control right
```

SDK接收机器狗下发的关节数据：

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

机器狗关节控制指令：

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

机器狗站立的简单demo：  
1.PreStanUp：调整姿势，将机器狗的腿收起来，为站立做准备；  
2.GetInitData：记录下当前时间与关节数据；  
3.StandUp：机器狗起立。

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



## 8 python版本

### 8.1 编译生成

python版本程序采用pybind的形式生成，需要在6.3的步骤中增加编译选项（默认添加）,之后编译

```
cmake .. -DBUILD_PYTHON=ON
make -j
```

正常情况下编译好的动态库文件会自动复制到 ***python/lib*** 目录下

### 8.2 运行demo

在 ***python***目录下 直接执行motion_example.py文件，效果与C++版本sdk一样

```
cd /python
python motion_example.py
```


###### &nbsp;

### 其他注意事项

1. X30运动主机是ARM架构的，如果开发者想在运动主机上编译自己的程序，需要注意。
2. WiFi通讯受网络环境干扰产生的通讯延迟波动，可能对控制频率在500Hz以上的控制器有一定影响。
