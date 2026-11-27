# Go2 仿真与导航

默认导航用 `jie_3d_nav` 的 A* 生成三维全局路径，由 SCAN 根据实时障碍生成局部路径，再由 D1 控制器跟踪。`go2_bringup` 将 PCD 栅格化为 OctoMap，并适配 `/goal_pose`、TF 和 Go2 速度仲裁器。

## 构建

需要 ROS 2 Jazzy、Gazebo Sim、`ros_gz`、`xacro`、`ros2_control` 和 `colcon`：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
source tools/setup_workspace.sh
colcon build --symlink-install --packages-up-to go2_bringup octo_planner
source tools/setup_workspace.sh
```

新增或重编译 ROS 包后，已打开的终端也要重新执行 `source tools/setup_workspace.sh`，更新当前包搜索路径。构建会包含 `scan_planner`，点云处理使用 `sensor_msgs_py`、NumPy 和 SciPy。

## 地图与启动

默认导航直接读取世界坐标系下的 PCD，`octomap_resolution` 默认 0.2 m。地图在 Git 忽略的 `tools/output/` 中：默认场景使用 `building_obstacles_5cm.pcd`，三层坡道使用 `building_accessible_5cm.pcd`。干净克隆按[地图工具说明](../tools/README.md)生成地图，并确保 PCD 与 Gazebo world 完全匹配。

默认场景：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true
```

指定地图：

```bash
ros2 launch go2_bringup sim.launch.py navigation:=true \
  pcd_path:=/absolute/path/to/map.pcd octomap_resolution:=0.2
```

三层坡道：

```bash
ros2 launch go2_bringup accessible.launch.py
```

无图形界面时增加 `use_rviz:=false gz_args:='-s -v 1'`。

## 目标与接口

在 RViz 使用 **3D Goal Pose**，或向 `/goal_pose` 发布 `geometry_msgs/msg/PoseStamped`。目标坐标系为 `world`，Z 值表示高度。适配器读取 Go2 TF，向 jie 规划器发送起终点；路径跟踪期间使用导航速度源，停止后切回遥控。

| 话题/服务 | 用途 |
| --- | --- |
| `/jie/octomap` | PCD 生成的静态 OctoMap |
| `/jie/planned_path` | 三维 A* 全局路径 |
| `/go2/obstacle_points` | 实时雷达过滤后的障碍点云 |
| `/scan/local_path` | SCAN 碰撞检查后的局部路径 |
| `/jie/tracking_point_marker` | 控制器当前跟踪点 |
| `/goal_pose` | RViz/外部目标输入 |
| `/cmd_vel_nav` | 导航速度输入 |
| `/cmd_vel` | 速度仲裁输出 |
| `/go2/set_navigation` | 遥控/导航模式切换 |

SCAN 以 5 Hz 生成局部路径。D1 使用全局目标判断到达，沿局部路径行驶。局部路径为空或超过 `local_path_timeout`（默认 1 秒）时输出零速度，有效局部规划恢复后继续。点云和里程计超过 0.5 秒会使 SCAN 清空输出。最终朝向调整也受局部碰撞检查约束。

RViz 的 `SCAN Local Path` 显示橙色局部路径。取消导航可发布 `/jie/stop_navigation` 的 `std_msgs/msg/Bool`，将 `data` 设为 `true`。

隔离集成验证在独立 ROS 域运行，覆盖绕障、阻塞停车、恢复、传感器超时、规划进程退出和目标结束。

```bash
source tools/setup_workspace.sh
ROS_DOMAIN_ID=81 python3 src/scan_planner/tests/test_navigation_integration.py
colcon test --packages-select scan_planner
colcon test-result --test-result-base build/scan_planner --verbose
```
