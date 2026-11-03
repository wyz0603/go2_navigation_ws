# go2_navigation_ws

Go2 Gazebo 仿真与导航工程。

ROS 2 Jazzy 工作区，支持遥控运动、静态 PLY / 实时点云全局规划和 Nav2 路径执行。直行、转向及返回路线已完成闭环复测；新增简化三层坡道场景已完成一层 → 二层 → 三层实际行走验证。原始 Building 的复杂楼梯及连续重复运行仍需进一步验证。

首次部署、完整启动流程、启动参数和各配置文件说明见[启动与配置详细说明](docs/startup_and_configuration_zh.md)。

## 构建

依赖 ROS 2 Jazzy、Gazebo、Nav2 和本机 ugv_nav4d 原生库：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wyz/ugv_nav4d/install/env.sh
cd /home/wyz/go2_navigation_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

每个新终端都需加载上述环境。原生库路径是本机依赖，迁移到其他机器需要调整。

## 启动与控制

遥控模式：

```bash
ros2 launch go2_bringup sim.launch.py
# 另一个交互终端：W/S 前后，A/D 横移，Q/E 转向，空格停止，Esc 退出
ros2 run go2_rl_controller keyboard_teleop
```

同时启动导航组件：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true
```

导航启动会同时打开 RViz2；无界面导航可追加 `use_rviz:=false`。

使用已生成的碰撞几何地图（当前已验证方式）：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true \
  map_file:=/home/wyz/go2_navigation_ws/tools/output/building_obstacles_5cm.ply
```

`map_file` 非空时，全局规划器加载 PLY 生成 MLS，实时雷达更新局部障碍地图。当前工作区存在 `building_obstacles_5cm.ply` 时默认加载它；显式传入 `map_file:=''` 才使用实时点云全局建图。PLY 必须与所启动的世界匹配，并使用相同的 `world` 坐标；不存在的文件会在启动时明确报错。

三层坡道场景使用独立入口，自动配套世界、地图和导航：

```bash
ros2 launch go2_bringup accessible.launch.py
```

该场景是新增的简化布局，原始 Building 保留。RViz 用 **3D Goal Pose** 选择楼层，地面 Z 分别为 0、3、6 米；2D Goal Pose 不能选择上层。详见[三层场景说明](docs/startup_and_configuration_zh.md#修复版三层导航场景2026-11-03)。

无界面运行追加 `gz_args:='-s -v 1'`。已有仿真时可单独启动 `navigation.launch.py`；不要重复启动导航或 `path_controller.launch.py`。

启动时速度来源为遥控；默认 `auto_execute_goal: true` 在收到有效目标时自动切换导航并执行。也可手动切换到导航或切回遥控：

```bash
ros2 service call /go2/set_navigation std_srvs/srv/SetBool '{data: true}'
ros2 service call /go2/set_navigation std_srvs/srv/SetBool '{data: false}'
```

仲裁节点是 `/cmd_vel` 的唯一发布者。模式切换清空旧指令；当前来源超过 0.3 秒无更新时输出零速度。速度上限为前后 0.5 m/s、横移 0.3 m/s、转向 0.8 rad/s。切回遥控只切断导航速度，不取消导航任务；再次切回导航前应先停止或确认原任务状态。不要向 `/cmd_vel` 直接发布命令。

向 `/goal_pose` 发布 `world` 坐标系的地面目标（`geometry_msgs/msg/PoseStamped`，一层地面 Z=0）。入口会从规划器配置读取 `distToGround`，自动加上机身离地高度。低层 `/ugv_nav4d_ros2/goal_pose` 接收的是机身高度目标；直接发送时地面 Z=0 应写成 Z=0.35，不能混用。默认规划成功后直接执行；只有设为 `auto_execute_goal: false` 的人工预览模式才需要调用 `/ugv_nav4d_ros2/execute_path`（`std_srvs/srv/Trigger`）。导航速度只有在导航模式下才会传入运动控制器。

地面层示例（先确认 `Loaded MLS from PLY` 且控制器 active；首次目标会触发规划器初始化，可能需要数秒）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
# 仅 auto_execute_goal: false 时，在确认规划成功后手动执行：
ros2 service call /go2/set_navigation std_srvs/srv/SetBool '{data: true}'
ros2 service call /ugv_nav4d_ros2/execute_path std_srvs/srv/Trigger '{}'
```

局部障碍先经 `obstacle_height_filter` 处理：使用点云采集时刻的 TF，按世界竖直方向保留相对 `base` 高度 -0.25～0.45 米的点，再根据可通行地图排除当前高度附近的坡面地面，输出 `/go2/obstacle_points`。地图中的未知区域和物体仍保留。独立的 `/go2/clearing_points` 使用高度受限的原始回波清除历史障碍，避免已排除的地面残留在 costmap 中。点云保留传感器坐标和时间戳；超过 0.5 秒或缺少对应 TF 的点云不会发布。

## 话题与坐标系

| 输入/输出 | 用途 |
| --- | --- |
| `/go2/points` | 近似 Mid360 原始点云，坐标系 `mid360_link` |
| `/go2/obstacle_points` | 按机身相对高度过滤的局部避障点云 |
| `/go2/clearing_points` | 用于清除历史障碍的地面及障碍回波 |
| `/odom`、`/tf` | Gazebo 真值定位，`world` → `base` |
| `/cmd_vel_teleop` | 遥控速度输入 |
| `/cmd_vel_nav` | Nav2 速度输入 |
| `/cmd_vel` | 仲裁后的 `geometry_msgs/msg/Twist` |
| `/follow_path` | Nav2 路径执行 action |
| `/goal_pose` | world 坐标系地面目标，自动转换为机身高度 |

规划器先把点云变换到世界坐标系，再裁剪并检查 MLS 地图边界。当前地图约 40 米见方，超出地图范围的点被丢弃。定位使用仿真真值，未接 SLAM；激光传感器为规则扫描近似模型，并非真实 Mid360 扫描模式。

`go2_bringup` 统一启动与参数；`go2_description`、`go2_gazebo`、`robot_sim_worlds` 提供模型和仿真；`go2_rl_controller`、`go2_rl_interfaces` 提供运动控制；`ugv_nav4d_ros2` 提供全局规划。`scan_planner` 是未接入当前导航链路的实验包，其目标话题不能用作导航目标。

## 检查

```bash
ROS_DOMAIN_ID=78 python3 -m unittest discover -s tests -v
ros2 lifecycle get /controller_server
ros2 action list -t
ros2 topic info /cmd_vel -v
```

仲裁测试覆盖来源隔离、切换清零、超时、限幅及非有限数值。复杂场景避障和模式切换后的任务恢复仍需闭环验证。

此前短程联调结果：全局目标距离容差收紧至 0.15 米、局部位置容差收紧至 0.08 米。启用相对高度过滤后，目标 `(1, 0, 0)` 执行成功（FollowPath SUCCEEDED），停止位置约 `(0.908, -0.055, 0.352)`，平面误差约 0.107 米；上次误差约 0.305 米。仍存在全局栅格化及运动控制误差，不能把 0.08 米局部容差等同于原始目标定位精度。详见 [验证记录](docs/static_navigation_validation.md)。

最新偏差修正复测：2 米目标、转向和返回均成功，停止后位置误差分别约 8.3、11.5、7.4 厘米。目标应发布到 `/goal_pose`，其 Z 表示地面高度；最终配置与证据见 [偏差修正记录](docs/navigation_bias_adjustment.md)。历史失败与点云中断结果保留在 [扩展验证记录](docs/navigation_extended_validation.md)。

三层坡道场景实际行走验证：二层、三层到达时机身高度分别为 3.343、6.346 米，平面误差约 18.3、7.1 厘米，均报告 `COMPLETED`。使用 `accessible.launch.py` 及其配套地图，详见 [三层行走验收](docs/navigation_diagnostics/accessible_motion_validation.md)。

## 世界碰撞几何导出

新增离线工具 [gazebo_world_to_pcd.py](tools/gazebo_world_to_pcd.py)，直接采样世界 SDF 及引用模型的碰撞几何，输出世界坐标下的 PCD：

```bash
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_obstacles.sdf \
  tools/output/building_obstacles.pcd \
  --bounds -20 -20 -0.2 20 20 8 --spacing 0.1
```

依赖安装、其他世界的模型路径和支持范围见 [tools/README.md](tools/README.md)。

转换为规划器支持的 PLY：

```bash
python3 tools/pcd_to_ply.py tools/output/building_obstacles.pcd
```

生成同目录 `building_obstacles.ply`，保留原始点坐标和采样密度。

当前推荐地图为 `tools/output/building_obstacles_5cm.ply`（约 140.6 万点，5 厘米采样）；MLS 规划栅格保持 20 厘米。重新生成：

```bash
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_obstacles.sdf \
  tools/output/building_obstacles_5cm.pcd \
  --bounds -20 -20 -0.2 20 20 8 --spacing 0.05 --max-samples 10000000
python3 tools/pcd_to_ply.py tools/output/building_obstacles_5cm.pcd
```

规划路径终点与请求目标的平面偏差超过 0.12 米时拒绝执行；最终姿态使用请求朝向。Nav2 进展检查同时考虑平移和转角，避免把终点转向当作完全没有进展。
