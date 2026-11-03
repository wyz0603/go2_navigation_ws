# Gazebo 世界碰撞几何 → PCD

`gazebo_world_to_pcd.py` 离线解析世界 SDF、递归展开本地模型引用，对 **collision 表面**采样并导出 XYZ PCD。无需启动 Gazebo 或 ROS，不使用 visual 网格，也不使用雷达扫描数据。

## 安装依赖

在工程根目录运行：

```bash
python3 -m pip install --target tools/.deps -r tools/requirements.txt
```

依赖安装到工具私有目录，不修改 ROS Python 环境。本机已配置依赖。也可以在自己的虚拟环境中安装 requirements。

## 转换当前场景

```bash
python3 tools/gazebo_world_to_pcd.py \
  src/robot_sim_worlds/worlds/building_obstacles.sdf \
  tools/output/building_obstacles.pcd \
  --bounds -20 -20 -0.2 20 20 8 \
  --spacing 0.1
```

示例结果：13 个碰撞体、324,380 个点。坐标是世界坐标，单位为米。输出默认是小端 float32 XYZ 二进制 PCD；加 `--ascii` 可导出文本 PCD。已有输出文件会被替换，转换失败不会覆盖旧文件。

其他场景：

```bash
python3 tools/gazebo_world_to_pcd.py /path/to/world.sdf output.pcd \
  --resource-path /path/to/models \
  --bounds -30 -30 -1 30 30 10 --spacing 0.05
```

| 参数 | 说明 |
| --- | --- |
| `--bounds` | `xmin ymin zmin xmax ymax zmax`，世界坐标下裁剪范围。包含无限平面时必须指定 |
| `--spacing` | 默认 0.05 米，控制面采样密度和去重体素边长；越小文件越大 |
| `--resource-path` | 额外模型搜索目录，可重复指定 |
| `--seed` | 默认 0，相同依赖版本及输入下结果可复现 |
| `--max-samples` | 默认 5,000,000，裁剪前面采样点总数上限，超出时报错 |
| `--ascii` | 输出文本 PCD |

采样按三角形面积进行随机分布，再按体素去重，因此 spacing 是近似密度参数，不保证每个位置的点间距或完全无孔洞。裁剪前的采样量按完整碰撞面面积估计；对巨大网格可调大 spacing 或 max-samples。

## 几何和坐标处理

- 支持 STL、DAE、OBJ 等 trimesh 可加载的三角网格，以及 box、sphere、cylinder、plane。
- 累积 world/model/link/collision 层级位姿；支持嵌套模型、include 的 pose 覆盖、mesh scale、以度表示的 RPY。
- include 按模型目录的 `model.config` 中首个 SDF 条目读取，缺少配置时使用 `model.sdf`。
- 支持相对/绝对路径、`file://`、`model://`、`package://`；自动搜索世界相邻的 models 目录、工程 src、Gazebo 资源环境变量及 AMENT 安装目录。
- SDF 的碰撞 plane 是无限平面，`size` 不是物理边界。工具在指定 bounds 内采样，不把地面错误地缩成视觉尺寸。
- 圆柱和球先近似为三角网格。包含遮挡面、背面和内部碰撞表面；结果是场景几何点云，不是激光可见点云，也不是占据栅格。
- 动态模型导出的是文件中的初始位姿，不包含仿真运行中的运动或后来生成的机器人。

当前不支持 `relative_to`、四元数 pose、placement_frame、merge include、mesh submesh、heightmap、远程 Fuel URI。遇到这些内容明确报错，需先转换为父级相对的普通 SDF/本地碰撞网格。不会静默跳过未支持的 collision。

PCD 只保存 XYZ，没有 ROS frame_id 字段；用于 ROS 时应明确指定世界坐标系。现有 ugv_nav4d 的文件加载配置使用 PLY，本工具不改变其加载格式。

## 验证

```bash
python3 -m unittest discover -s tools/tests -v
```

覆盖 include 位姿覆盖、父级旋转、link/collision 偏移、mesh 缩放、visual 排除、无限平面范围、随机采样复现、采样预算、循环引用及 PCD 二进制/文本读回。

## PCD 转 PLY（供 ugv_nav4d 文件加载使用）

`pcd_to_ply.py` 调用 PCL 转换程序，不重新采样、不调整坐标或分辨率。依赖 `pcl-tools`（本机已安装）：

```bash
# 其他机器缺少依赖时安装：sudo apt-get install pcl-tools
python3 tools/pcd_to_ply.py tools/output/building_obstacles.pcd
# 自定义输出路径或使用文本格式：
python3 tools/pcd_to_ply.py input.pcd output.ply --ascii
```

支持 PCL 可读取的 ASCII、binary、binary_compressed PCD。默认输出同名二进制 PLY；已有同名输出在转换成功后替换，失败时保留。导出不包含 camera 元素，坐标保持原样。

当前生成 `tools/output/building_obstacles.ply`，324,380 个点；已逐点检查 XYZ 与原始 PCD 完全一致。规划器加载该文件需要设置 `load_mls_from_file: true`、`mls_file_type: ply` 和 `mls_file_path` 的绝对路径。脚本只转换文件，不修改当前导航运行配置。

## 三层场景与实际导航验证

`build_accessible_world.py` 生成独立的 `building_accessible.sdf`：三层平台、两段连续坡道及侧护栏，可视几何与碰撞一致，保留原始 Building 文件。修改几何后重新导出对应地图，不能配用原始建筑的 PLY：

```bash
python3 tools/build_accessible_world.py
python3 tools/gazebo_world_to_pcd.py src/robot_sim_worlds/worlds/building_accessible.sdf tools/output/building_accessible_5cm.pcd --spacing 0.05 --max-samples 10000000
python3 tools/pcd_to_ply.py tools/output/building_accessible_5cm.pcd
colcon build --packages-select robot_sim_worlds go2_bringup --symlink-install
source install/setup.bash
ros2 launch go2_bringup accessible.launch.py
```

在另一个已加载环境的终端运行 `python3 tools/verify_accessible_navigation.py`，默认依次前往地面高度 0、3、6 米的目标。该脚本会驱动仿真机器人，应在独占当前导航任务时运行；它核对执行状态和实际里程计高度，结果写入 `/tmp/go2_navigation_verification/report.json`，失败时调用停止服务。可用 `--goal X Y GROUND_Z` 指定一个或多个目标，`--output` 设置记录目录，`--timeout` 设置每个目标的最长墙钟秒数。
