# Go2 仿真与导航

默认导航用 `jie_3d_nav` 的 A* 生成三维全局路径，由 D1 控制器跟踪路径。`go2_bringup` 将 PCD 栅格化为 OctoMap，并适配 `/goal_pose`、TF 和 Go2 速度仲裁器。

## 构建

需要 ROS 2 Jazzy、Gazebo Sim、`ros_gz`、`xacro`、`ros2_control` 和 `colcon`：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
source tools/setup_workspace.sh
colcon build --symlink-install --packages-up-to go2_bringup octo_planner
source tools/setup_workspace.sh
```

新增或重编译 ROS 包后，已打开的终端也要重新执行 `source tools/setup_workspace.sh`，更新当前包搜索路径。工作区不再构建旧 PCT/SCAN 规划包，因此导航构建不需要 GTSAM 或 OSQP。

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
| `/jie/tracking_point_marker` | 控制器当前跟踪点 |
| `/goal_pose` | RViz/外部目标输入 |
| `/cmd_vel_nav` | 导航速度输入 |
| `/cmd_vel` | 速度仲裁输出 |
| `/go2/set_navigation` | 遥控/导航模式切换 |

D1 节点负责跟踪全局路径，不会根据实时雷达重新规划局部绕障。动态障碍和实体机器人运行仍需在目标环境单独验证。
