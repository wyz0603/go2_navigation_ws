# 场景与 PCD 地图工具

默认导航将 PCD 直接加载为 OctoMap，不生成 PCT tomogram。

## 导出场景碰撞点云

`gazebo_world_to_pcd.py` 离线解析 SDF 的 collision 几何并采样为 XYZ PCD，不需要启动 Gazebo：

```bash
python3 -m pip install --target tools/.deps -r tools/requirements.txt
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_obstacles.sdf \
  tools/output/building_obstacles_5cm.pcd \
  --bounds -20 -20 -0.2 20 20 8 --spacing 0.05 --max-samples 10000000
```

PCD 保存世界坐标下的 collision 表面点，没有 `frame_id` 字段。`5cm` 表示采样间距；导航 OctoMap 默认分辨率为 0.2 m。采样属于表面离散化，不能视作实体封闭网格。

其他 SDF 可使用：

```bash
python3 tools/gazebo_world_to_pcd.py /path/to/world.sdf tools/output/map.pcd \
  --resource-path /path/to/models --bounds -30 -30 -1 30 30 10 --spacing 0.05
```

## 三层坡道地图

```bash
python3 tools/build_accessible_world.py
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_accessible.sdf \
  tools/output/building_accessible_5cm.pcd \
  --spacing 0.05 --max-samples 10000000
ros2 launch go2_bringup accessible.launch.py
```

地图必须与启动的 Gazebo world 使用同一坐标系和场景版本。默认及三层场景 PCD 都保存在 Git 忽略的 `tools/output/`。
