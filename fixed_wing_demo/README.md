# 固定翼无人机自动起飞与 8 字飞行仿真

本工程运行于 Ubuntu 26.04、ROS 2 Lyrical、Gazebo Jetty 10.5。已经编译并在本机运行验证。

## 一条指令启动

本机新开的 Bash 终端已自动加载工作空间，直接运行：

```bash
ros2 launch fixed_wing_demo flight.launch.py
```

会自动打开 Gazebo、导入跑道和飞机、启动桥接和控制节点、开始物理仿真。飞机自动地面加速、拉起、爬升至约 60 米，然后持续循环飞 8 字。相机在启动后自动切换到追飞机视角；按 Esc 可以解除跟随。地面的青色线表示 8 字参考轨迹的投影。

当前已打开的旧终端可先加载环境；以下整行也能直接启动：

```bash
source /opt/ros/lyrical/setup.bash && source "/home/liangzhang/zl_ws/chatgpt_ws/1 install-ros-start/ros2_ws/install/setup.bash" && ros2 launch fixed_wing_demo flight.launch.py
```

正常实时仿真约 3 秒开始拉起，约 30 秒达到巡航高度；接着捕获航迹。8 字每圈约 75–85 秒。机器负载会影响实际等待时间。Ctrl+C 停止整个 launch。只运行一个本工程的默认 launch 实例。

## 模型和物理实现

飞机外观使用 [PX4/PX4-gazebo-models 的 RC Cessna](https://github.com/PX4/PX4-gazebo-models/tree/5577035667afb4b63fe1f966fb1a58bbb05d905b/models/rc_cessna)，全部 DAE 资源已放在 `models/rc_cessna/`，无需联网。原始模型和 BSD-3-Clause 许可保留，来源记录位于 `third_party/`。

本工程的 `models/fixed_wing/model.sdf` 将模型外观放大为 2 倍，采用 3 kg 质量、等效 0.8 m² 翼面积以及简化的起落架接触。它没有直接运行原始模型的 PX4 插件。

`src/fixed_wing_dynamics.cpp` 是独立的简化六自由度飞行动力学插件：根据空速和迎角计算升力、阻力、失速衰减及侧滑阻尼，施加螺旋桨推力和等效控制力矩；重力、地面碰撞和刚体运动由 Gazebo 物理引擎积分。控制没有写入飞机位姿或速度，没有使用运动轨迹插件。没有模拟风、真实舵机或电池；适用于任务控制演示，并非真实飞机的辨识模型或 PX4/ArduPilot SITL。

## 控制脚本

`scripts/autopilot.py` 通过 ROS 里程计反馈完成闭环控制：

1. TAKEOFF_ROLL：全油门沿跑道加速至约 13 m/s。
2. ROTATE：给出抬头指令，离地超过 2 m 后进入爬升。
3. CLIMB：油门 PI 调速；升力前馈及垂直速度反馈维持爬升。
4. FIGURE_EIGHT：沿参数曲线 `x=1200+220*sin(t)`、`y=110*sin(2*t)` 进行带前视距离的航迹追踪，通过滚转形成转弯，持续计圈。

曲线上的前后邻域搜索保留轨迹相位，避免在 8 字交叉点跳到另一条支路。机体采用 FLU、世界采用 ENU：数学正滚转产生右转，数学负俯仰为抬头。里程计的速度在机体系；控制器转到世界系后计算航向和爬升率。

内环是等效舵面力矩控制，没有运行飞控固件。无控制输入超过 1 秒仿真时间时，动力学插件切断推力。

## 话题与记录

launch 默认使用 ROS_DOMAIN_ID=42，Gazebo Transport 使用每次启动独立的 partition，避免与其他示例场景混淆。

```bash
ROS_DOMAIN_ID=42 ros2 topic echo /fixed_wing/status
ROS_DOMAIN_ID=42 ros2 topic echo /fixed_wing/odometry --once
```

| 话题 | 消息 | 用途 |
|---|---|---|
| `/fixed_wing/command` | geometry_msgs/Twist | linear.x：油门 0–1；angular.x/y：滚转、俯仰角指令，弧度 |
| `/fixed_wing/odometry` | nav_msgs/Odometry | 50 Hz 左右的机体状态 |
| `/fixed_wing/status` | std_msgs/String | 飞行阶段、高度、速度、圈数、误差 |
| `/fixed_wing/reference_path` | nav_msgs/Path | 参考 8 字轨迹，可在 RViz 显示 |
| `/fixed_wing/flown_path` | nav_msgs/Path | 实际飞行轨迹 |
| `/clock` | rosgraph_msgs/Clock | Gazebo 仿真时钟 |

遥测默认保存为 `~/.ros/fixed_wing_demo/flight.csv`，每次启动会覆盖此文件。需要保留不同运行时，请传入 `telemetry_file:=/absolute/path/flight.csv`。

## launch 参数

```bash
ros2 launch fixed_wing_demo flight.launch.py gui:=false real_time_factor:=8
ros2 launch fixed_wing_demo flight.launch.py altitude:=80
ros2 launch fixed_wing_demo flight.launch.py ros_domain_id:=43
```

默认渲染器为 Ogre，兼容本机 Intel 显卡。更改高度会保留原有水平航迹；高度越高，捕获航迹前的爬升距离越长。

## 重建与验证

在 ROS 工作空间根目录 `ros2_ws` 中：

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select fixed_wing_demo
source install/setup.bash
ros2 launch fixed_wing_demo flight.launch.py
```

本机加速仿真已验证多圈闭环飞行。测试读取真实 Gazebo 里程计遥测，并检查起飞阶段、完整圈数、巡航高度/速度、航迹误差，以及飞机实际访问两个 8 字环。

```bash
python3 src/fixed_wing_demo/tests/validate_flight.py /absolute/path/flight.csv
```

运行需记录至少两个完整循环。验证结果及轨迹图放在本机工程的 `logs/` 和 `results/` 中。

ROS/Gazebo 桥接包当前版本在停止进程时可能记录退出阶段的段错误；已验证运行期间的桥接和飞行正常。
