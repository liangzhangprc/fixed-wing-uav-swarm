# PX4 固定翼软件在环仿真

PX4 源码已下载到 ROS 工作空间 `ros2_ws/src/PX4-Autopilot`，版本固定为官方稳定版 **v1.17.0**，提交 **d6f12ad1c4f70ad3230afd7d86e971421e02fef4**。SITL 可执行文件已编译到 `build/px4_sitl_fixedwing/bin/px4`。

## 一条 launch 指令启动

本机新终端执行：

```bash
ros2 launch fixed_wing_demo px4_sitl.launch.py
```

Gazebo 会自动打开并加载原工程的飞机和跑道。ROS 节点 `px4_sitl` 启动并管理独立的原生 PX4 固件进程；仿真桥接节点供给传感器并接收执行器输出；任务节点在 PX4 接受 Offboard 并通过正常解锁检查后，自动执行起飞、爬升至约 60 米和 8 字循环。

原来的 `flight.launch.py` 保留，仍用于不接入飞控固件的演示。SITL 使用独立 ROS_DOMAIN_ID=43 和独立 Gazebo Transport partition。

```bash
ros2 launch fixed_wing_demo px4_sitl.launch.py gui:=false
ROS_DOMAIN_ID=43 ros2 topic echo /px4/sitl_status
ROS_DOMAIN_ID=43 ros2 topic echo /px4/process_status
ROS_DOMAIN_ID=43 ros2 node list
```

启动后约 12–18 秒进入 Offboard 并正常解锁；随后开始地面加速。未绕过 PX4 解锁检查，也不使用强制解锁。Ctrl+C 停止整个 launch，ROS supervisor 会关闭它启动的 PX4 进程。

## 软件在环数据链

```text
Gazebo 刚体物理 / IMU 250 Hz / 真实位置和空速
      ↓ ros_gz_bridge + px4_mavlink_bridge.py
TCP 127.0.0.1:4560: HIL_SENSOR + HIL_GPS
      ↓
原生 PX4 SITL: 传感器驱动 → EKF2 → 固定翼姿态/角速度控制 → 控制分配
      ↓ HIL_ACTUATOR_CONTROLS
ROS /fixed_wing/actuator_command → Gazebo 舵面力矩与螺旋桨推力
```

任务节点发送前视航迹控制产生的姿态和油门目标，通过 UDP 14540 / MAVLink `SET_ATTITUDE_TARGET` 输入 PX4 Offboard。SITL 下 Gazebo 动力学插件只接收归一化执行器输出，没有使用原工程的姿态 PD 内环。PX4 的 EKF2 估计姿态用于固件的姿态/速率闭环；外部 ROS 航迹节点当前仍读取 Gazebo 里程计，作为任务层的理想定位来源。

执行器映射：PWM_MAIN_FUNC1=Servo1（滚转），FUNC2=Servo2（俯仰），FUNC3=Servo3（偏航），FUNC4=Motor1（油门）。HIL 输出前三项为 -1 至 1 的舵面命令，第四项为 0 至 1 的油门。

仿真桥实现 ENU/FLU 与 NED/FRD 的转换，并输入 IMU、磁场、气压、差压空速和 GPS。IMU 注入很小的可重复噪声，避免 PX4 对完全不变的量化传感器数据报 STALE。传感器和执行器不是通过 ROS DDS 直接连接 PX4；这里使用 PX4 官方支持的 MAVLink simulator API，适配原工程的自定义 Gazebo 动力学。

## 文件

- `launch/px4_sitl.launch.py`：新 SITL launch。
- `scripts/px4_node.py`：独立固件进程的 ROS 节点，负责日志和关闭。
- `scripts/px4_mavlink_bridge.py`：传感器、执行器和 Offboard 接口。
- `config/px4_rcS`：在运行目录的飞机配置中加载参数；不修改 PX4 原始 airframe。
- `config/px4_sitl_fixedwing.px4board`：SITL 编译配置，启用 simulator_mavlink / pwm_out_sim，关闭不需要的原生 gz_bridge 和 DDS 客户端。
- `config/px4_bridge.yaml`：ROS/Gazebo 话题桥接。
- `src/fixed_wing_dynamics.cpp`：新增 actuator 模式，直接把 PX4 的归一化舵面和油门输出转换为气动力矩及推力。

PX4 是独立的 CMake 工程，源码及其 Python 环境均有 COLCON_IGNORE，不参与 ROS 包扫描。Python 依赖放在工作空间 `px4_venv` 中，避免 PX4 所需 empy 3 与 ROS 的 empy 4 冲突。

## 编译与重现

本机已安装 `ninja-build`、`python3-venv`、`python3-pip`、`libxml2-utils`，并已完成 PX4 和 ROS 工程编译。

工程根目录的 `build_px4_sitl.sh` 可重建完整集成；首次运行会下载指定版本和所需子模块。它不编译 NuttX 硬件固件，也不下载无关的外部仿真器。

```bash
./build_px4_sitl.sh
source /opt/ros/lyrical/setup.bash
source "ros2_ws/install/setup.bash"
ros2 launch fixed_wing_demo px4_sitl.launch.py
```

不同源码位置可指定 `px4_root:=/absolute/path/PX4-Autopilot`。该目录需有 `build/px4_sitl_fixedwing/bin/px4`，Python 环境仍使用本 ROS 工作空间中的 `px4_venv`。

## 日志与诊断

每次运行在 `~/.ros/fixed_wing_demo/sitl_<launch进程PID>/` 建立独立目录：

- `flight.csv`：任务/飞行轨迹遥测。
- `px4/px4-console.log`：PX4 控制台日志。
- `px4/log/`：PX4 ULog 飞行日志。
- `px4/romfs/`：本次运行复制的 ROMFS/airframe 配置。

桥接统计与执行器记录位于本次运行目录的 `px4_bridge.csv`，与飞行和固件日志一并保留。

可在运行期间调用固件自己的诊断命令：

```bash
ros2_ws/src/PX4-Autopilot/build/px4_sitl_fixedwing/bin/px4-listener vehicle_attitude -n 1
ros2_ws/src/PX4-Autopilot/build/px4_sitl_fixedwing/bin/px4-listener vehicle_local_position -n 1
ros2_ws/src/PX4-Autopilot/build/px4_sitl_fixedwing/bin/px4-commander status
```

默认使用 PX4 实例 0，TCP 4560 和 UDP 14540。启动前应停止其他占用同一 PX4 实例或端口的 SITL。Gazebo 使用实时速率 1，当前传感器桥没有实现 Gazebo/PX4 的严格 lockstep。

模型仍为简化的六自由度固定翼气动力，舵面以等效力矩建模，未增加真实舵机、风或电池放电模型。PX4 固件和估计/控制代码是真实运行的，并非外观模型或模拟固件接口。

官方依据：[PX4 仿真和 MAVLink simulator API](https://docs.px4.io/main/en/simulation/)、[v1.17.0 源码](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0)。

## 本机验收

无界面连续运行约 332 秒，完成 3 圈；Gazebo 图形界面运行约 256 秒，完成 2 圈。均通过正常解锁、地面滑跑、起飞、爬升与循环航迹验证。稳定航迹段高度约 60.25 米、速度约 18 米/秒，平均横向误差约 2.7 米；运行期间 PX4 保持 Offboard，没有 failsafe。轨迹 CSV 和验收统计保存在工程根目录 `results/px4-*`。

本机 ros_gz_bridge 在关闭阶段仍出现既有的 SIGSEGV（运行阶段桥接正常）；Gazebo、任务节点和 PX4 固件已正常退出。该信息保留在验证日志中。

六机集群新增 `px4_swarm.launch.py`，详见 [README_SWARM.md](README_SWARM.md)。为修复多机测试中确认的 POSIX 定时队列竞态，当前固件额外应用了 `config/px4_posix_hrt.patch`，重建脚本会自动应用。
