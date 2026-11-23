# 工程布局与导航数据流

## 源码

```text
go2_navigation_ws/
├── src/
│   ├── go2_bringup/       PCD→OctoMap、Go2 适配和启动文件
│   ├── go2_description/   Go2 模型与机器人描述
│   ├── go2_gazebo/        Gazebo 插件和 ROS 桥接
│   ├── go2_rl_controller/ Go2 策略控制与速度仲裁
│   ├── go2_rl_interfaces/ 控制消息接口
│   ├── jie_map_msgs/      jie_3d_nav 地图消息
│   ├── octo_planner/      jie_3d_nav 三维 A* 与 D1 路径跟踪
│   └── robot_sim_worlds/  Gazebo 场景
├── tools/                 场景导出与 PCD 地图生成
└── docs/                  构建、启动与地图说明
```

`src/` 中有 8 个 ROS 包，均直接放在 `src/<package>/`。旧 PCT 与 SCAN 包及对应导航入口已从工作区移除。

## 导航数据流

```text
静态 PCD → OctoMap → jie_path_node 三维 A* → D1 路径跟踪
                                                ↓
Go2 策略控制器 ← 速度仲裁器 ← /cmd_vel_nav
```

A* 和路径跟踪器来自 `6-robot/jie_3d_nav`；`jie_map_msgs` 提供消息类型。Go2 适配层读取 `world → base` TF，把 RViz 目标转换为规划输入。PCD 加载节点位于 `go2_bringup`，避免 tomogram 转换链路。

D1 控制器跟踪全局路径，不提供基于实时雷达的局部动态绕障。当前静态点云地图和路径结果需要与仿真世界坐标一致。

## 地图

`tools/output/` 被 Git 忽略，干净克隆按[地图工具说明](../tools/README.md)重新生成地图。

| 文件 | 用途 |
| --- | --- |
| `building_obstacles_5cm.pcd` | 默认建筑世界 |
| `building_accessible_5cm.pcd` | 三层坡道世界 |

文件名中的 `5cm` 表示 PCD 采样间距；OctoMap 分辨率默认 0.2 m。导航可用 `pcd_path:=/absolute/path/to/map.pcd` 指定其他地图。

## 构建与启动

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
source tools/setup_workspace.sh
colcon build --symlink-install --packages-up-to go2_bringup octo_planner
source tools/setup_workspace.sh
ros2 launch go2_bringup sim.launch.py navigation:=true
```

三层坡道使用 `ros2 launch go2_bringup accessible.launch.py`。
