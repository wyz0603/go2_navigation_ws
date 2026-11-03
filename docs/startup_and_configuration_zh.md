# Go2 仿真与导航启动、配置说明

本文按当前工作区代码说明环境准备、编译、仿真和导航启动，以及常用配置的修改方法。默认工作区路径和外部规划库路径来自当前开发机；迁移到其他机器时请替换为实际路径。

## 1. 工程组成

这是一个 ROS 2 Jazzy 工作区，主要包含以下功能包：

| 功能包 | 作用 |
| --- | --- |
| `go2_description` | Go2 URDF/Xacro 机器人模型及关节控制器定义 |
| `go2_gazebo` | Gazebo 与 ROS 之间的时钟、点云、IMU、里程计和 TF 桥接配置 |
| `robot_sim_worlds` | 建筑、楼梯、空场景等 Gazebo 世界及模型资源 |
| `go2_rl_controller` | ONNX 步态控制器、键盘遥控和速度命令仲裁 |
| `go2_rl_interfaces` | RL 控制状态接口 |
| `go2_bringup` | 仿真、规划器、Nav2 路径控制的统一启动入口和参数 |
| `ugv_nav4d_ros2` | 全局地形/路径规划 ROS 2 节点；依赖外部原生 `ugv_nav4d` 库 |
| `scan_planner` | 独立实验规划器，不属于本文介绍的 Go2 导航闭环 |

默认仿真世界为 `building_obstacles.sdf`。导航可以使用实时点云构图，也可以加载世界坐标系下预先生成的 PLY 地图。当前导航定位使用 Gazebo 提供的仿真真值 TF，不包含 SLAM。

## 2. 运行环境与依赖

当前工程按以下环境配置：

- Ubuntu 上的 ROS 2 Jazzy。
- Gazebo Sim 及 ROS-Gazebo 集成包，包含 `ros_gz_sim`、`ros_gz_bridge`。
- Nav2 Controller、生命周期管理器和 Regulated Pure Pursuit 插件。
- `xacro`、`robot_state_publisher`、`controller_manager`、`ros2_control` effort controller。
- Python 依赖：NumPy、PyYAML，以及 ROS 包 `sensor_msgs_py`、`tf2_ros_py`。
- 外部 `ugv_nav4d` 原生库及其 ROS 环境脚本。当前 README 使用 `/home/wyz/ugv_nav4d/install/env.sh`；其他机器需要安装该库并使用对应路径。
- `colcon` 构建工具。

若当前 ROS 安装不含工程依赖，可在安装 rosdep 并初始化后，从工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # 仅在本机尚未初始化 rosdep 时执行一次
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`ugv_nav4d` 是外部依赖，需依照其项目说明单独编译安装；`rosdep` 不会替代该原生库的安装。Gazebo 的发行版和 ROS 版本需相互匹配。

## 3. 编译工作区

新终端中依次加载 ROS、外部规划库环境，然后编译：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wyz/ugv_nav4d/install/env.sh
cd /home/wyz/go2_navigation_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果外部库脚本不在上面的路径，请改成实际安装位置。工作区有变更后重新执行 `colcon build --symlink-install`，然后在每个新终端重新 source 环境。可用以下命令确认 ROS 能找到主要包：

```bash
ros2 pkg prefix go2_bringup
ros2 pkg prefix ugv_nav4d_ros2
```

### 每个新终端需要做什么

所有启动、CLI 检查和控制终端都必须加入相同的 ROS 环境。推荐在每个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wyz/ugv_nav4d/install/env.sh
source /home/wyz/go2_navigation_ws/install/setup.bash
```

若不同终端的 `ROS_DOMAIN_ID` 不一致，节点之间无法互相发现；使用多机或多工作区时还要确认 Domain ID、网络和 ROS middleware 配置一致。

## 4. 启动 Gazebo 仿真

在已加载环境的终端运行：

```bash
ros2 launch go2_bringup sim.launch.py
```

此命令会启动 Gazebo、机器人状态发布器、ROS-Gazebo bridge、机器人实体、关节控制器、ONNX 控制器和 `/cmd_vel` 仲裁器。仿真启动时先暂停世界，初始化控制器后再自动解除暂停。默认不启动导航。

使用 `navigation:=true` 时会同时打开 RViz2，并加载工程为 Go2 仿真配置的导航视图（Fixed Frame 为 `world`，显示规划地图/路径、TF 和机器人模型）。如果只需后台运行导航，可追加 `use_rviz:=false`。

### 无界面运行

```bash
ros2 launch go2_bringup sim.launch.py gz_args:='-s -v 1'
```

`-s` 仅关闭 Gazebo 图形界面；是否打开 RViz 由 `use_rviz` 单独控制。完全无界面导航时同时传入 `navigation:=true use_rviz:=false gz_args:='-s -v 1'`。

### 仿真启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `navigation` | `false` | 是否同时启动导航栈；取 `true` 时包括规划器和 Nav2 路径控制器 |
| `use_rviz` | `true` | 导航启动时打开 RViz2；设置 `false` 可关闭 |
| `map_file` | 存在时为 `tools/output/building_obstacles_5cm.ply`，否则为空 | 静态 PLY 全局地图；显式传入空字符串才强制使用实时点云全局建图 |
| `world` | 安装空间内 `building_obstacles.sdf` | Gazebo 世界 SDF 文件路径 |
| `world_name` | `building_obstacles` | Gazebo world 名称，用于启动后解除暂停；换世界时应与 SDF 内 `<world name="...">` 相符 |
| `x`, `y`, `z` | `0.0`, `0.0`, `0.34` | 机器人生成位置，单位米 |
| `yaw` | `0.0` | 机器人初始偏航角，单位弧度 |
| `gz_args` | `-v 3` | 传给 Gazebo 的参数，例如 `-s -v 1` |
| `controller_config` | `go2_rl_controller/config/policy_controller.yaml` | ONNX 控制器参数 YAML 路径 |

例如换到楼梯世界（先核对该 SDF 中 `<world>` 的名字）：

```bash
ros2 launch go2_bringup sim.launch.py \
  world:=/home/wyz/go2_navigation_ws/src/robot_sim_worlds/worlds/stairs.sdf \
  world_name:=stairs_course
```

启动实体位置需要避开碰撞几何。自定义世界引用的模型还必须能通过 Gazebo 资源路径找到。

## 5. 键盘遥控

仿真启动后，在另一个已加载环境的交互式终端运行：

```bash
ros2 run go2_rl_controller keyboard_teleop
```

默认键位：`W/S` 前进/后退，`A/D` 横移，`Q/E` 左右转，空格发送停止，`Esc` 退出。键盘程序发布 `/cmd_vel_teleop`。停止或退出前建议按空格；命令仲裁器还会在遥控输入超过 0.3 秒未更新时自动输出零速度。

## 6. 启动导航

### 方式 A：与仿真一起启动

默认导航（本工作区存在默认 PLY 时加载该地图）：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true
```

明确使用实时点云全局建图：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true map_file:=''
```

加载预制地图：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true \
  map_file:=/home/wyz/go2_navigation_ws/tools/output/building_obstacles_5cm.ply
```

`map_file` 必须指向存在的 `.ply` 文件。规划器启动时会检查扩展名和文件是否存在。地图中的点必须处于该 Gazebo 世界的 `world` 坐标系，且与运行世界一致。工程内当前可用示例文件为 `tools/output/building_obstacles.ply` 和 `tools/output/building_obstacles_5cm.ply`。

加载预制 PLY 时，全局规划器从文件生成 MLS，全局规划不再依赖实时点云建图；实时点云仍经高度过滤后提供给 Nav2 局部避障。显式传入 `map_file:=''` 可从 `/go2/points` 实时构建规划地图。

实时点云模式在完成地图更新后，按约 1 秒的墙钟间隔发布 MLS 和可通行性地图，供 RViz 和健康节点读取。当前工作区若存在 `tools/output/building_obstacles_5cm.ply`，启动默认使用它作为稳定全局地图；显式传入 `map_file:=''` 才启用实时点云全局建图。实时点云仍用于 Nav2 局部避障。旧进程可通过 `/ugv_nav4d_ros2/map_publish` 临时补发地图。

### 方式 B：仿真已运行时单独启动导航

先确保已有仿真只启动了一份，再在新终端运行：

```bash
ros2 launch go2_bringup navigation.launch.py
```

该方式也会默认启动 RViz2；关闭界面可指定 `use_rviz:=false`。

或指定 PLY：

```bash
ros2 launch go2_bringup navigation.launch.py \
  map_file:=/home/wyz/go2_navigation_ws/tools/output/building_obstacles_5cm.ply
```

此入口启动地面目标适配器、Nav2 `controller_server`、局部障碍高度过滤器、路径跟随客户端、运行状态汇总节点及全局规划器。状态汇总节点会向 RViz 操作面板提供执行就绪状态。由于该入口期望 Gazebo 的 `/clock`、`/tf`、`/odom` 和点云已经在运行，不应在仿真尚未启动时单独使用。不要同时用 `navigation:=true` 和单独的 `navigation.launch.py` 启动两份规划器或路径控制器。

## 7. 发送目标并执行

推荐通过 `/goal_pose` 发布以 `world` 为坐标系的**地面目标**。目标位置的 Z 表示地面高度，平地通常为 `0.0`。适配器从 `planner.yaml` 读取 `distToGround`（当前为 0.35 米），将它加到 Z 后转发给规划器。目标朝向使用四元数。

当前 bringup 默认启用 `auto_execute_goal`：收到有效地面目标后自动切换到导航速度源；规划成功且生成有效路径后立即发送给跟随器。规划失败、地图缺失或目标无效时不会执行。若需恢复人工预览确认，可在 `planner.yaml` 中将 `auto_execute_goal` 设为 `false`。

示例：请求平地上的 `(1, 0)` 目标，朝向 0 弧度：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

仅在将 `auto_execute_goal` 设为 `false` 的人工预览模式下，查看规划器输出及 RViz 路径，确认规划成功后切换到导航速度源并执行：

```bash
ros2 service call /go2/set_navigation std_srvs/srv/SetBool '{data: true}'
ros2 service call /ugv_nav4d_ros2/execute_path std_srvs/srv/Trigger '{}'
```

导航模式下，控制器速度才会被接受。切回遥控源：

```bash
ros2 service call /go2/set_navigation std_srvs/srv/SetBool '{data: false}'
```

切换来源会清除之前缓存的速度。切回遥控仅切断导航速度，不会自动取消规划或执行任务；执行中切换模式后，恢复导航前先确认任务状态，必要时调用停止服务：

```bash
ros2 service call /ugv_nav4d_ros2/stop_execution std_srvs/srv/Trigger '{}'
```

`/cmd_vel` 由仲裁器独占发布，不要直接向它发送遥控或导航指令。键盘遥控和 Nav2 分别发布到 `/cmd_vel_teleop`、`/cmd_vel_nav`。

> 低层规划器目标话题 `/ugv_nav4d_ros2/goal_pose` 的 Z 是机身高度，不是地面高度。若跳过适配器直接发布，平地目标通常应将 Z 设为 `0.35`。一般使用 `/goal_pose` 即可，避免混淆两种约定。

## 8. 地图生成与加载

工程提供 Gazebo 世界碰撞几何导出工具。它读取 SDF 中的静态 collision 几何并导出世界坐标点云，不需要启动仿真。完整支持范围、依赖和参数见 [Gazebo 世界导出工具说明](../tools/README.md)。

例如生成 5 厘米采样 PCD，再转换为规划器支持的 PLY：

```bash
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_obstacles.sdf \
  tools/output/building_obstacles_5cm.pcd \
  --bounds -20 -20 -0.2 20 20 8 --spacing 0.05 --max-samples 10000000
python3 tools/pcd_to_ply.py tools/output/building_obstacles_5cm.pcd
```

转换后将生成同名 `.ply` 文件。`map_file` 使用 PLY 绝对路径最明确。规划器的 MLS 栅格默认分辨率为 0.2 米；提高输入点密度不等于提高规划栅格分辨率。

## 9. 配置文件说明

修改源码中的 YAML 后需重新构建对应包并重新启动相关节点。通常只改 `config` 文件即可；如通过 launch 参数传入其他 YAML，要确认节点参数名与 YAML 根节点名称对应。

### 9.1 `go2_bringup/config/planner.yaml`

规划器、目标适配器和路径跟随客户端的基础配置。

| 参数 | 当前值 | 用途 / 调整提示 |
| --- | ---: | --- |
| `world_frame`, `robot_frame` | `world`, `base` | 世界及机器人 TF frame；必须与仿真 TF 一致 |
| `auto_execute_goal` | `true` | 有效目标规划成功后自动切换导航速度源并执行；设为 `false` 使用人工预览确认 |
| `load_mls_from_file` | `false` | 文件加载开关；`navigation.launch.py` 会在 `map_file` 非空时覆盖为 true |
| `mls_file_type`, `mls_file_path` | `ply`, 空 | 文件类型和路径；正常通过 launch 的 `map_file` 设置 |
| `grid_resolution` | `0.2` | MLS 地图栅格分辨率，米 |
| `dist_min/max_x/y/z` | XY ±20，Z -2～8 | 规划空间边界，覆盖当前静态地图中的多层楼面 |
| `distToGround` | `0.35` | 地面到机身参考高度；同时用于 `/goal_pose` 高度适配 |
| `robotSizeX/Y` | `1.1 / 1.1` | 规划占用尺寸，包含 Nav2 半径 0.35 米及每侧 0.20 米跟踪余量，避免路径贴住坡道护栏 |
| `maxSlope`, `maxStepHeight` | `0.45 / 0.45` | 最大可通行坡度参数及台阶高度，需结合机器人和场景验证 |
| `minTraversablePercentage` | `0.4` | 可通行区域判定比例 |
| `translationSpeed`, `rotationSpeed` | `0.5`, `0.5` | 规划模型中的平移及旋转速度 |
| `maxTime` | `20.0` | 单次搜索时间上限，秒 |
| `numThreads` | `4` | 搜索线程数 |
| `goalDistanceMargin` | `0.08` | 规划器目标距离余量 |
| `max_goal_xy_error` | `0.12` | 路径终点相对目标的 XY 偏差上限；超过时拒绝执行 |
| `align_goal_orientation` | `true` | 最终路径朝向对齐请求目标 |
| `initialPatchRadius` | `1.0` | 初始地形 patch 尺寸相关参数 |
| `mls_gap_size` | `0.4` | MLS 中断裂/间隙处理参数 |

该 YAML 中还包含倒车、曲率代价、角度采样、Reeds-Shepp 终端路径等搜索参数。修改搜索行为时建议一次调整少量参数，并同时检查规划时间和路径质量。`distToGround`、frame 名称、地图边界以及实际世界坐标尤其需要保持一致。

### 9.2 `go2_bringup/config/controller.yaml`

包含 Nav2 控制器和障碍高度过滤器配置。

| 参数 | 当前值 | 用途 |
| --- | ---: | --- |
| `controller_frequency` | `20.0` Hz | Nav2 控制循环频率 |
| `progress_checker.required_movement_radius` | `0.05` m | 进展检查的平移量 |
| `progress_checker.required_movement_angle` | `0.10` rad | 进展检查的角度量 |
| `progress_checker.movement_time_allowance` | `15.0` s | 最长无进展时间 |
| `goal_checker.xy_goal_tolerance` | `0.08` m | Nav2 终点 XY 容差 |
| `goal_checker.z_goal_tolerance` | `0.25` m | 机身高度容差；使用 `go2_bringup::HeightAwareGoalChecker`，避免相同 XY 的上下层目标被提前判定到达 |
| `goal_checker.yaw_goal_tolerance` | `0.15` rad | Nav2 终点朝向容差 |
| `FollowPath.desired_linear_vel` | `0.25` m/s | 路径跟随期望线速度 |
| `FollowPath.lookahead_dist` | `0.6` m | 前视距离 |
| `FollowPath.lookahead_time` | `1.2` s | 当前启用速度自适应前视；以 0.25 m/s 行走时取最小前视距离 0.3 米，减少转弯切角 |
| `FollowPath.use_collision_detection` | `true` | 启用控制器碰撞检查 |
| 局部 costmap `width/height` | `6 / 6` m | 滚动局部代价地图范围 |
| 局部 costmap `resolution` | `0.05` m | 局部代价地图栅格分辨率 |
| `robot_radius` | `0.35` m | 局部代价地图机器人半径 |
| `obstacle_layer.lidar.topic` | `/go2/obstacle_points` | 过滤后的局部障碍点云输入 |
| `obstacle_layer.clearing.topic` | `/go2/clearing_points` | 仅用于清除历史障碍，不新增障碍标记 |
| `obstacle_max_range/raytrace_max_range` | `3.0 / 4.0` m | 障碍标记及清除距离 |
| `min_relative_height/max_relative_height` | `-0.25 / 0.45` m | 相对机器人 base 高度过滤范围 |
| `terrain_tolerance` | `0.10` m | 点到可通行地形拟合面的高度容差 |

高度过滤在世界竖直方向计算相对 `base` 的高度，并使用点云采集时刻对应的 TF。匹配 TF 暂不可用时等待；点云超过 0.5 秒或时间异常时会丢弃。调整高度上下限会改变局部障碍物检测对象，需结合机器人自身点云和场景实测。

过滤器订阅 `/ugv_nav4d_ros2/trav_map`，只移除当前高度附近、与可通行地形平面吻合的地面点；未知区域、非可通行单元和高于坡面的物体仍参与避障。`ground_clearance`、`max_step_height` 从 `planner.yaml` 的 `distToGround`、`maxStepHeight` 统一读取，不能将其他楼层当作当前地面。清障回波的高度范围为相对 base 的 `-(ground_clearance + max_step_height)` 到 `max_relative_height`，避免跨楼层射线错误清除障碍。未收到地形地图时保留原高度过滤行为。

路径跟随客户端也从 `planner.yaml` 读取 `robot_frame: base`，用于路径偏差及分段终点的 TF 查询；改机器人模型时应与规划器及局部 costmap 的机身 frame 保持一致。

### 9.3 `go2_rl_controller/config/policy_controller.yaml`

ONNX 低层步态控制器配置，由 `sim.launch.py` 启动。

| 参数 | 当前值 | 用途 |
| --- | ---: | --- |
| `control_rate` | `200.0` Hz | 控制器更新频率 |
| `policy_decimation` | `4` | 策略推理间隔相关参数 |
| `action_scale` | `0.25` | 策略输出动作缩放 |
| `kp`, `kd` | `25.0`, `0.0` | 关节位置控制增益；当前 Gazebo 关节动力学已提供阻尼 |
| `command_scale` | `[0.5, 1.0, 1.0]` | 输入速度命令缩放 |
| `command_timeout` | `0.3` s | 速度指令超时 |
| `sensor_timeout` | `0.2` s | 传感器数据超时 |
| `heading_hold` | `true` | 直行时抑制航向漂移 |
| `heading_kp` | `1.0` | 航向保持比例增益 |
| `heading_max_yaw_rate` | `0.25` rad/s | 航向保持角速度上限 |
| `joint_state_topic`, `imu_topic` | `/joint_states`, `/go2/imu` | 传感器输入话题 |
| `cmd_vel_topic` | `/cmd_vel` | 仲裁后的速度输入 |
| `effort_topic` | `/go2_effort_controller/commands` | 关节力矩输出 |

策略模型路径由 launch 中的 `policy` 文件确定，当前为包内 `policy/policy.onnx`。修改步态参数可能导致动作不稳定，特别是动作缩放、关节默认角度、增益、动作映射和电机力矩曲线参数。

### 9.4 `go2_description/config/ros2_controllers.yaml`

控制器管理器更新频率当前为 200 Hz，使用 `joint_state_broadcaster` 和 `effort_controllers/JointGroupEffortController`。机器人 URDF 中的关节名称必须与此 YAML 的 12 个关节名称一致。

### 9.5 `go2_gazebo/config/bridge.yaml`

定义 Gazebo 和 ROS 话题映射，主要包括 `/clock`、`/go2/points`、`/go2/imu`、`/odom` 和 `/tf`。更换机器人模型或 Gazebo 话题名称后，需同步修改桥接配置及对应消费节点。

## 10. 速度源仲裁和限制

`command_mux` 是 `/cmd_vel` 的唯一发布者：

| 模式 | 输入话题 | 选择服务 |
| --- | --- | --- |
| 遥控 | `/cmd_vel_teleop` | `/go2/set_navigation` 请求 `data: false` |
| 导航 | `/cmd_vel_nav` | `/go2/set_navigation` 请求 `data: true` |

速度限幅为前后 `±0.5 m/s`、横移 `±0.3 m/s`、转向 `±0.8 rad/s`。当前模式以外来源的消息会被丢弃；切换模式会清零缓存；当前来源超过 0.3 秒没有更新时输出零速度。监视器使用系统单调时钟，因此仿真暂停时 watchdog 仍会工作。

## 11. 常用检查命令

检查节点、话题和控制器：

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel -v
ros2 topic echo /go2/points --once
ros2 topic echo /tf --once
ros2 lifecycle get /controller_server
ros2 action list -t
```

导航运行中可查看规划器状态和路径：

```bash
ros2 topic list | grep ugv_nav4d_ros2
ros2 topic echo /ugv_nav4d_ros2/planner_status
ros2 topic echo /ugv_nav4d_ros2/execution_status
```

话题或服务的实际类型可用 `ros2 topic info -v <topic>`、`ros2 service type <service>` 查询。人工预览模式下检查规划结果，再调用 `execute_path`；默认自动执行模式无需重复调用。

## 12. 常见问题排查

| 现象 | 检查方向 |
| --- | --- |
| `ros2 launch` 找不到包 | 确认 source 顺序正确；包有修改时重新 `colcon build`；检查 `ros2 pkg prefix <包名>` |
| 找不到 `ugv_nav4d` 共享库/头文件 | 当前终端是否 source 外部库的 `install/env.sh`；外部库是否已编译安装且架构匹配 |
| Gazebo 找不到模型或纹理 | 检查 `world` 路径、SDF include、本机模型资源，以及 Gazebo 资源路径 |
| Gazebo 窗口打开但仿真不动 | 看启动日志中的控制器 spawner 和 `gz service .../control`；确认 `world_name` 与世界文件中的 world 名一致 |
| 机器人控制器未 active | `ros2 control list_controllers`；检查 `controller_manager`、关节名称和仿真是否运行 |
| 点云话题没有数据 | 检查 `/go2/points`、Gazebo bridge 日志、`/clock`；确认 Gazebo 世界已解除暂停 |
| Nav2 报 TF 时间过旧 | 确认仿真 `/clock` 正常且相关节点使用仿真时间；不要混用系统时间与仿真时间 |
| 过滤后没有 `/go2/obstacle_points` | 检查点云时间戳、frame_id，以及对应采集时刻的 `world`→传感器和 `world`→`base` TF |
| PLY 启动报错 | 确认文件存在、扩展名 `.ply`、路径可读；地图是否与当前世界及 `world` 原点一致 |
| 有路径但机器人不走 | 检查 `/go2/set_navigation` 是否为 true、`/cmd_vel_nav` 是否有消息、`/cmd_vel` 是否被仲裁器发布、控制器是否 active |
| 坡道前报 `detected collision ahead` | 检查地形过滤器是否收到 `trav_map`、局部 costmap 是否把坡面留作障碍、规划路径与护栏是否有足够跟踪余量；保留碰撞检查 |
| 上层选点变成一层 | 使用 3D Goal Pose，旋转视角后点击目标楼层的可见地面；2D Goal Pose 始终投影到 z=0 |
| RViz 只显示模型或地图缺一部分 | 检查规划器进程及共享库、PLY 与世界配套关系、`MultiLayeredSurfaceMap`/`TraversabilityMap` 订阅和状态；关闭 MLS 高度过滤后缩小视图 |
| 键盘没有响应 | 使用交互式终端运行键盘程序；确认当前模式是遥控且没有另一个程序抢占终端输入 |
| 节点互相不可见 | 检查所有终端的 `ROS_DOMAIN_ID`、ROS 环境和网络配置 |

## 13. 安全停止与当前验证边界

### RViz 执行按钮的状态

导航入口统一启动 `field_operations.py`，它提供操作面板所需的健康状态。该节点晚启动时也会读取已经发布的规划状态、执行状态、路线风险及区域设置。

| 按钮文字 | 含义与操作 |
| --- | --- |
| `Start new path` | 已有可执行的新路线，检查预览后点击执行 |
| `Start: waiting for health` | 尚未收到健康报告，检查状态汇总节点是否运行 |
| `Start: check readiness` | 存在未通过的就绪检查，查看 Readiness 明细 |
| `Start: plan a route first` | 尚未生成新的预览，用 3D Goal Pose 设置目标；2D Goal Pose 仅适用于 z=0 地面 |
| `Start: use Resume` | 已有暂停任务，点击 Resume current path 继续 |
| `Start: rebuilding maps` | 正在重建地图，等待完成 |
| `Start: mission running` | 已有任务执行中 |
| `Start: route invalid` | 当前路线无效，需要重新规划 |

默认自动执行模式会选择导航速度源；人工预览模式在执行前通过 `/go2/set_navigation` 手动选择。重新启动仿真后，应重新设置目标，旧进程的暂停任务不会自动恢复。

### 退出与残留话题

统一使用 launch 启动导航，在该终端按 Ctrl+C 会关闭它管理的节点。单独从其他终端执行的 `ros2 run` 需要在对应终端退出。不要为了启用面板再启动第二份 `field_operations.py`。

`ros2 topic list` 也包含仅有订阅者的话题。退出后先用 `ros2 node list`、`ros2 topic info <topic> -v` 确认是否还有节点和端点；节点已退出而列表仍过时时，可执行 `ros2 daemon stop` 刷新发现缓存。该命令不会停止仿真节点。

遥控时按空格发零速度；停止导航任务可调用 `/ugv_nav4d_ros2/stop_execution`，再切回遥控模式。退出仿真时使用 launch 终端的 `Ctrl+C`。

当前记录显示直行、转向和返回路线完成过闭环复测，新增简化坡道场景完成了一层到二层、三层的实际行走；原始 Building 楼梯、下行、不同世界、连续重复运行和模式切换后的任务恢复仍需进一步验证。仿真使用规则扫描近似点云与真值定位，不代表真实 Mid360 扫描模式或真实机器人定位精度。详细测试过程见 [静态导航验证](static_navigation_validation.md)、[偏差修正记录](navigation_bias_adjustment.md)、[扩展验证记录](navigation_extended_validation.md) 和 [三层行走验收](navigation_diagnostics/accessible_motion_validation.md)。

### 导航库加载失败与多层选点

若日志出现 `libugv_nav4d.so: cannot open shared object file`，规划器已退出，模型和控制器仍可能继续显示。重新编译 `ugv_nav4d_ros2` 后，执行 `source install/setup.bash`；工作区现在通过环境钩子加载构建时找到的外部规划库路径。

当前 Building 模型和 PLY 包含多个高度的表面。`inspect_traversability` 查询只返回已生成的可通行性节点，某一位置只匹配到一层并不证明 PLY 没有上层。上层目标失败时，需要检查对应楼梯和楼层的可通行连通性。3D Goal Pose 现在在没有拾取到表面时提示重新选点，不再静默发布 z=0 的目标；仅点击不拖动时使用默认朝向，避免沿用上次的朝向。

## 修复版三层导航场景（2026-11-03）

原始 Building 的复杂碰撞表面未直接覆盖。新增 `building_accessible.sdf` 是简化修复场景：0、3、6 米三层地面，两段宽 3 米、水平长度 14 米的连续坡道，坡度约 12.1°，楼层由平台连接。可视几何与碰撞几何一致。它改变了建筑布局，并非原建筑外观不变的补丁。

坡道两侧带碰撞护栏，出入口保持开放，防止规划器从坡道侧边跨台阶进入。全局规划采用 1.1×1.1 米占用尺寸，为局部控制的 0.35 米机器人半径预留跟踪余量；未放宽 `maxSlope` 或 `maxStepHeight`，控制器碰撞检查仍开启。

启动修复场景（先关闭其他仿真实例）：

```bash
cd ~/go2_navigation_ws
source install/setup.bash
ros2 launch go2_bringup accessible.launch.py
```

此入口同时指定 `building_accessible.sdf`、`building_accessible` 世界名称以及 `building_accessible_5cm.ply`，避免地图错配。原来的 `sim.launch.py navigation:=true` 仍启动原场景，不要用它验证修复场景。

RViz 使用 **3D Goal Pose** 在可见的目标楼层地面选点并拖动方向。旋转视角露出楼层后再选，俯视时上层会遮住下层。标准 2D Goal Pose 仍投影到 z=0，不能用于选择楼层。

在 Displays 面板确认 `Global Options → Fixed Frame` 为 `world`，勾选 `MultiLayeredSurfaceMap` 和 `TraversabilityMap`，并保持前者的 `Height Filter` 关闭；面板隐藏时通过 `Panels → Displays` 打开。默认使用斜视视角。鼠标滚轮缩放、左键拖动旋转、中键拖动平移；缩小视图查看完整三层，再旋转到能看到目标地面的角度。实时 `PointCloud2` 只是当前雷达回波，不能用它判断整张静态导航地图是否加载完整。默认自动执行开启，选点后无需再点执行；`execution_status` 应从 `EXECUTING` 变为 `COMPLETED`。

通过 `/goal_pose` 发布地面目标时，二层 Z=3.0，三层 Z=6.0；适配器自动加机身离地高度。例如三层目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.839, y: 1.423, z: 6.0}, orientation: {w: 1.0}}}'
```

复现几何和地图生成：

```bash
python3 tools/build_accessible_world.py
python3 tools/gazebo_world_to_pcd.py src/robot_sim_worlds/worlds/building_accessible.sdf tools/output/building_accessible_5cm.pcd --spacing 0.05 --max-samples 10000000
python3 tools/pcd_to_ply.py tools/output/building_accessible_5cm.pcd
colcon build --packages-select robot_sim_worlds go2_bringup --symlink-install
```

启动此场景后，在另一个加载环境的终端做实际行走验收：

```bash
source install/setup.bash
python3 tools/verify_accessible_navigation.py --output /tmp/go2_navigation_verification
cat /tmp/go2_navigation_verification/report.json
```

脚本依次发送一层、二层、三层地面目标，需要等待数分钟，运行时不要同时发送其他目标。它要求跟随器报告完成，并检查实际 `/odom` 的平面误差和机身高度，不能只以规划成功或执行按钮状态判定通过。`report.json` 记录各目标结果，`odometry.jsonl` 每 10 秒记录一次位置、楼层目标和速度；失败时请求停止执行并返回非零退出码。自定义目标可重复传入 `--goal X Y GROUND_Z`，例如 `--goal 0.839 1.423 6.0`。

已完成一轮实际行走验收：从初始位置依次到达一层、二层、三层目标，执行状态均为 `COMPLETED`。二层、三层机身实际高度分别为 3.343、6.346 米，平面误差约 18.3、7.1 厘米。详细位置、耗时、原始记录和 RViz 截图见 [三层行走验收](navigation_diagnostics/accessible_motion_validation.md)。原始 Building 楼梯及下行、连续重复运行尚未完成验收。
