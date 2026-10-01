# 六机 PX4 固定翼 SITL 集群

在本机新终端运行以下命令，默认打开 Gazebo 可视化，自动部署六架飞机、启动六个独立 PX4 固件进程，并持续执行起飞、爬升和 8 字循环：

```bash
ros2 launch fixed_wing_demo px4_swarm.launch.py gui:=true
```

可视化沿用本机已验证的 Ogre 渲染器。六架飞机采用不同颜色，场景有六条平行起飞道；相机跟随第三架飞机，使用较大的观察偏移以查看集群。需要细看单机时，可以在 Gazebo 中缩放/旋转视角。

## 航迹与避碰

六架飞机共享原工程的水平 8 字路径：中心东向位置 1200 米，两个半轴参数 220/110 米，巡航速度 18 米/秒。集群采用垂直分层：目标高度依次为 60、78、96、114、132、150 米；起飞道间隔 18 米。各机独立沿相同水平路线飞行，不要求六机位于同一个高度或锁定完全相同的航迹相位。

`swarm_coordinator.py` 以 20 Hz 检查全部 15 对飞机，使用三维相对位置和速度预测未来 6 秒内的最近接近距离。

- 8 米：当前安全距离门限。
- 12 米：预测接近预警门限；解除预警使用额外 3 米迟滞。
- 3 米：保守碰撞包络门限，侵入时计数并输出错误日志。这是机体中心距离判据，不等同于 Gazebo 接触传感器记录。
- 出现风险时，低编号飞机降低目标高度/速度，高编号飞机提高目标高度/速度；高度最大调整 6 米，速度调整 1 米/秒。整体高度目标保持有序，至少间隔 12 米，目标变化经过限速。

避碰节点输出的是任务高度/速度目标，任务控制器将其转换为 Offboard 姿态与油门目标。PX4 自身的 EKF2、固定翼姿态/角速度控制和执行器分配仍然实际运行，Gazebo 插件接收的是 PX4 执行器输出。没有直接移动飞机模型或绕过 PX4 控制。

常规分层飞行本身避免了水平 8 字交叉处的冲突；动态预测用于发现偏离分层后的额外接近风险。当前使用 Gazebo 真值作为任务层定位，线性预测与分层策略属于本仿真场景的避碰方法，未验证风扰、飞控故障或真实传感器误差下的安全保证。

## 独立实例与通信

| 飞机 | PX4 实例 | MAVLink 系统 ID | 仿真 TCP | Offboard UDP | 巡航高度 |
|---|---:|---:|---:|---:|---:|
| uav_1 | 0 | 1 | 4560 | 14540 | 60 m |
| uav_2 | 1 | 2 | 4561 | 14541 | 78 m |
| uav_3 | 2 | 3 | 4562 | 14542 | 96 m |
| uav_4 | 3 | 4 | 4563 | 14543 | 114 m |
| uav_5 | 4 | 5 | 4564 | 14544 | 132 m |
| uav_6 | 5 | 6 | 4565 | 14545 | 150 m |

每架飞机有独立的 ROS 命名空间、传感器/执行器话题、固件运行目录和参数文件。默认 ROS_DOMAIN_ID=44；Gazebo Transport 使用每次启动独立的 partition。原单机 launch 保留。不要同时运行占用实例 0–5 或这些端口的其他 PX4 仿真。

```bash
ROS_DOMAIN_ID=44 ros2 topic echo /swarm/status
ROS_DOMAIN_ID=44 ros2 topic echo /uav_1/px4/sitl_status
ROS_DOMAIN_ID=44 ros2 topic echo /uav_6/status
```

PX4 多实例命令使用 `--instance`，例如运行期间：

```bash
ros2_ws/src/PX4-Autopilot/build/px4_sitl_fixedwing/bin/px4-commander --instance 5 status
```

端口和系统 ID 规则参考 PX4 自带 `px4-rc.mavlinksim`、`px4-rc.mavlink`、`rcS`，以及 [PX4 多机仿真说明](https://docs.px4.io/main/en/sim_gazebo_classic/multi_vehicle_simulation)。实际世界仍使用现有 Gazebo Jetty 自定义动力学，不依赖 Gazebo Classic。

## 验证与日志

无界面验收命令：

```bash
ros2 launch fixed_wing_demo px4_swarm.launch.py gui:=false stop_after_one_lap:=true
```

六架全部完成至少一圈后自动退出。常规可视化命令没有该验证参数，会持续循环；Ctrl+C 停止所有固件与 ROS 节点。

已完成六机无界面验收，仿真时间约 147.25 秒，每机完成一圈，六个 PX4 都保持已解锁/Offboard，运行期间无 failsafe。最小机间距离 **17.959 米**，独立通过六份轨迹 CSV 插值重算得到相同结果；碰撞包络事件与安全距离违规均为 **0**。正常分层运行没有触发动态避碰干预；迎头接近、垂直分层交叉、远离/静止与三机冲突目标分配分别通过单元测试。

验收文件在工程根目录 `results/swarm-validation/`：`validation.json`、`swarm_summary.json`、`separation.csv` 和六机各自的 `flight.csv`、`px4_bridge.csv`、`px4-console.log`。

每次启动会建立 `~/.ros/fixed_wing_demo/swarm_<launch进程PID>/`：世界、模型、桥接配置、实例清单、机间距离记录、各机轨迹与完整 PX4 日志均按次保存。

## PX4 POSIX 定时队列修复

本项目使用 PX4 v1.17.0。在六机首轮测试中发现 POSIX HRT 定时队列重入竞态：回调解锁期间，其他线程可能将同一任务重新入队；回调返回后重复插入使链表自循环，造成控制任务停顿。调试确认了队列节点指向自身，并记录在 `logs/px4-hrt-queue-diagnosis.log`。

`config/px4_posix_hrt.patch` 在持锁重新入队前移除已有节点，保留单次入队。补丁已应用并重新编译；工程根目录 `build_px4_sitl.sh` 会自动应用该补丁，重复运行会检查已应用状态。PX4 控制算法未修改。

统一退出已检查，六个 PX4 固件和自编写 ROS 节点均正常结束。本机现有 `ros_gz_bridge` 有时在关闭阶段报告 SIGSEGV；运行期桥接与上述验收正常。多机与单机均保留此限制说明。
