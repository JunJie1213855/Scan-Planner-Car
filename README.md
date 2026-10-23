<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
</div>

<p align="center">
  <img src="assets/images/demo1.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo2.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo3.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

> 二编：增加一个map.pcd，并提供测试例子（非搭建gazebo测试）

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成。



## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

---

### 1.1、在一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```
### 1.2、在另一个终端启动自定义确定性仿真器与规划器（Mode 1 二维闭环演示，机身高度保持不变）：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```
---


### 2.1、在一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```
### 2.2、使用 `map.pcd` 运行 Mode 3 跨层开环演示(注意替换自己的路径)：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=/your_map_location/map.pcd \
  reference_path_file:=/your_project_location/src/planner/plan_manage/config/reference_path.map.yaml
```

示例参考路径从 `(-5.5, 5.5, 0.10)` 沿地图坡道上升到
`(-5.5, -4.5, 1.55)`。路径文件中的 `z` 是地面/路线高度，规划器会再加上
`grid_map.body_height`（默认 `0.4 m`），因此目标机身高度约为 `1.95 m`。


RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



### 导航模式说明
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `initial_path` 话题获取全局路径，并在局部范围内进行避障

控制器模式包括 `open_loop`（开环）、`closed_loop`（原闭环）和 `closed_loop_car`（差速小车闭环）。

本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map` 和 `pcd_map_file`。

与原作者实现保持一致，`closed_loop` 通过平面 `cmd_vel` 跟踪 `x/y/yaw`，
适用于二维仿真或真机底盘接口；多楼层仿真应使用 `open_loop`，直接按规划得到的
三维 B-spline 发布里程计。Mode 3 默认从 `(-5.5, 5.5, 0.5)` 启动，Mode 1
默认从 `(-19.0, 1.0, 0.25)` 启动，仍可用 `init_x`、`init_y`、`init_z` 覆盖。

Mode 3 默认等待外部节点发布 `/initial_path`。也可以通过
`reference_path_file` 启动仓库内的演示发布器；该发布器会等待首个
`body_pose` 和规划器订阅者就绪后发布一次路径。路径至少需要两个 xyz 点，
相邻点会按三维距离 `0.5 m` 降采样，并始终保留最终点。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

实际硬件部署时，激光惯导里程计（LIO）、相机和宇树（Unitree）驱动均为外部依赖，默认启动会将规划器输入映射到 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 话题以及 RealSense 对齐深度图话题，可根据实际安装的驱动栈修改话题重映射配置。

## 地面差速小车适配

在 ROS 2 移植基础上，本分支增加了差速小车专用控制器、实际路径进度反馈，以及可配置的规划参考高度偏移。地面小车可以使用原有 B-spline 规划结果进行平面闭环跟踪。

### 差速小车控制器

新增 `closed_loop_controller_car` 节点，使用空间 Pure Pursuit 跟踪 B-spline 路径，仅输出 `cmd_vel.linear.x` 和 `cmd_vel.angular.z`，侧向速度保持为零。控制器支持转弯限速、加减速限制、朝向误差较大时先原地转向，以及到达终点后停车；里程计超时、轨迹无效或跟踪偏差超限时停止输出运动指令。

在已构建并加载工作空间的终端中，启动二维仿真：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop_car car_odom_frame:=world
```

已有实机里程计和感知输入时，可使用：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop_car car_odom_frame:=odom
```

`car_odom_frame` 必须与输入里程计的 `header.frame_id` 一致，以上实机示例假定为 `odom`。启动入口默认传入 `world`，覆盖控制器配置文件中的 `odom_frame: odom`。这个参数只用于检查坐标系名称，不执行坐标变换；轨迹和里程计的位置必须使用一致的坐标系。使用 `/clock` 仿真时钟时追加 `use_sim_time:=true`。

控制参数位于 `src/planner/plan_manage/config/controllers.yaml` 的 `closed_loop_controller_car.ros__parameters` 下：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `max_vx` / `max_vyaw` | `0.5 m/s` / `1.0 rad/s` | 前向速度和偏航角速度上限 |
| `max_accel` / `max_decel` | `0.5` / `0.8 m/s²` | 前向加速和减速限制 |
| `lookahead_min` / `lookahead_max` | `0.25` / `0.6 m` | Pure Pursuit 前视距离范围 |
| `rotate_enter` / `rotate_exit` | `0.7` / `0.25 rad` | 原地转向的进入和退出阈值 |
| `finish_dist` | `0.1 m` | 终点距离容差 |
| `max_tracking_error` | `0.8 m` | 跟踪偏差上限 |
| `odom_timeout` | `0.5 s` | 里程计超时时间 |

控制器通过 `planning/execution_progress` 发布 `scan_planner_msgs/msg/ExecutionProgress`，包含轨迹编号、实际路径进度、终点状态和跟踪丢失状态。选择 `closed_loop_car` 时，启动入口自动启用 `fsm.use_controller_progress`，规划状态机按反馈更新轨迹执行时间；原地转向或反馈中断时不再单纯按墙钟推进轨迹，跟踪丢失时请求重规划。单独启动规划器和小车控制器时，需自行给规划器设置 `fsm.use_controller_progress: true`。

原有 `open_loop` 和 `closed_loop` 模式仍可使用。差速控制器按 XY 平面跟踪，不控制 Z；自带二维运动学仿真也不会模拟小车爬坡或跨层运动。

### 规划参考高度偏移

地面小车的里程计原点可能位于地面附近，直接作为规划参考点时，可能落入地面占据栅格或其膨胀区域。新增 `fsm.odom_height_offset` 参数，替代原先硬编码的 `0.25 m` 偏移：

```text
规划参考高度 = 输入里程计 z + fsm.odom_height_offset
```

在 `src/planner/plan_manage/config/planner.yaml` 中配置：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.odom_height_offset: 0.25 # 单位 m；无需偏移时设为 0.0
```

该偏移沿地图 Z 轴应用于规划器内部位置，影响轨迹起点、碰撞查询和自膨胀显示；不修改原始里程计、点云或地图，也不会让小车实际升高。参数在启动时读取，要求为有限数值；修改配置后重新启动规划器生效。若里程计原点已经位于合适的机体参考高度，应设为 `0.0`，避免重复抬高。例如输入 `z=0.0 m`、偏移 `0.25 m` 时，规划参考高度为 `0.25 m`；输入已为 `z=0.25 m` 时，相同偏移会得到 `0.50 m`。

目标高度需与这个参考点一致：

- `navi_mode:=1`：手动目标使用首次收到的里程计高度加偏移后的值，并在后续保持该高度。
- `navi_mode:=2`：预设航点使用 `fsm.waypoints` 中给定的 Z，不会自动叠加里程计高度偏移。
- `navi_mode:=3`：参考路径高度按“输入路径 Z + `grid_map.body_height`”处理；需与里程计加偏移后的参考高度对应。

`fsm.odom_height_offset` 与 `grid_map.body_height` 分别处理里程计和参考路径，不能直接视为同一个参数；应根据各自输入的高度含义配置。偏移量还应结合小车实际尺寸、地图分辨率和垂直膨胀参数选择。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型包括 `scan_planner_msgs/msg/Bspline`、`scan_planner_msgs/msg/DataDisp` 和差速小车执行反馈 `scan_planner_msgs/msg/ExecutionProgress`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

参考路径演示发布器也可以独立运行：

```bash
ros2 run scan_planner reference_path_publisher.py --ros-args \
  --params-file src/planner/plan_manage/config/reference_path.map.yaml \
  -r body_pose:=/quad_0/body_pose -r initial_path:=/initial_path
```

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
