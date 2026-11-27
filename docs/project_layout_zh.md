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
│   ├── jie_octomap/       OctoMap 导入与编辑工具
│   ├── jie_map_msgs/      jie_3d_nav 地图消息
│   ├── octo_planner/      jie_3d_nav 三维 A* 与 D1 路径跟踪
│   ├── scan_planner/      SCAN B 样条局部避障与碰撞检查
│   └── robot_sim_worlds/  Gazebo 场景
├── tools/                 场景导出与 PCD 地图生成
└── docs/                  构建、启动与地图说明
```

`src/` 中有 10 个 ROS 包，均直接放在 `src/<package>/`。

## 导航数据流

```text
静态 PCD → OctoMap → jie_path_node 三维 A*
                              ↓ 全局路径
实时雷达 → 障碍点云过滤 → SCAN 局部规划 → D1 路径跟踪
                                            ↓
Go2 策略控制器 ← 速度仲裁器 ← /cmd_vel_nav
```

A* 和路径跟踪器来自 `6-robot/jie_3d_nav`；`jie_map_msgs` 提供消息类型。Go2 适配层读取 `world → base` TF，把 RViz 目标转换为规划输入。PCD 加载节点位于 `go2_bringup`，避免 tomogram 转换链路。

SCAN 根据实时雷达和全局路径生成经过碰撞检查的局部路径，D1 跟踪局部路径并判断全局目标到达。空路径、过期路径或传感器失效会触发停车。静态地图、局部路径和仿真 TF 使用一致的世界坐标系。

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
