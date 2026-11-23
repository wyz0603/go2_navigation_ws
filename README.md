# Go2 导航工程

ROS 2 Jazzy 仿真与导航工作区。默认导航使用 `jie_3d_nav` 的三维 A* 全局规划器和 D1 路径跟踪器：PCD 地图加载为 OctoMap，Go2 适配层负责目标、TF 和速度模式。

## 启动

```bash
source tools/setup_workspace.sh
ros2 launch go2_bringup sim.launch.py navigation:=true
```

三层坡道场景：

```bash
ros2 launch go2_bringup accessible.launch.py
```

默认地图在 `tools/output/building_obstacles_5cm.pcd`，三层场景地图在 `tools/output/building_accessible_5cm.pcd`。地图生成见[工具说明](tools/README.md)。启动时可用 `pcd_path:=/path/to/map.pcd` 指定地图。

## 工程结构

- `src/<package>/`：8 个 ROS 包，直接单层放置
- `tools/`：Gazebo 场景导出与 PCD 地图生成
- `docs/`：构建、启动与地图说明

## 文档

- [启动、参数与构建](docs/startup_and_configuration_zh.md)
- [工程布局与导航数据流](docs/project_layout_zh.md)
