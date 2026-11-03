# 静态 PLY 导航验证（2026-10-30，更新）

## 最新结果：相对高度过滤与收紧容差

- 新增 `obstacle_height_filter.py`，按采集时间的 TF 和机身相对世界竖直高度过滤，阈值 -0.25～0.45 米。点云仍用原始传感器 frame/stamp。
- 局部地图订阅 `/go2/obstacle_points`，观测更新期限为 0.5 秒；过滤丢弃超过 0.5 秒的采集数据。
- 全局 goalDistanceMargin 从 0.5 调为 0.15 米，局部 xy_goal_tolerance 从 0.25 调为 0.08 米。
- 相同目标 `(1, 0, 0)`：FOUND_SOLUTION、FollowPath 接收成功，约 14 秒后 `Reached the goal!`、状态 4、All paths executed。
- 停止位置约 `(0.9078, -0.0549, 0.3517)`，相对原始目标的平面误差约 0.107 米。
- 新增单元测试覆盖相同几何在不同楼层高度的过滤结果、俯仰传感器下的世界竖直高度筛选；与现有仲裁测试共 4 项通过。
- 实测仍仅为平地短路径，未据此证明坡道、楼梯、动态避障通过。
- 静态地图规划器在首次目标时初始化，应等待 MLS 加载和控制器 active 后发送目标，再等待规划成功。不能在发送首次目标前一直等待 Planner state: Ready。

## 上次结果（固定世界高度过滤）

配置：building_obstacles 世界、默认生成位置、0.1 米采样 PCD 转换的 PLY、world 真值定位、0.2 米 MLS、实时点云局部代价地图。

启动参数：`navigation:=true map_file:=/home/wyz/go2_navigation_ws/tools/output/building_obstacles.ply gz_args:='-s -v 1'`。

验证结果：

- PLY 加载成功，MLS 为 200×200 格，未对世界地图另加坐标偏移。
- `/go2/points` 的订阅者只有局部代价地图，全局规划器使用文件地图。
- `/cmd_vel_nav` 从 controller_server 发往 command_mux。
- 世界目标 `(1, 0, 0)`，规划返回 FOUND_SOLUTION，生成 17 个路径位姿。
- FollowPath 接收成功，约 5.6 秒后报告到达；返回 SUCCEEDED（4），客户端报告全部路径执行完毕。
- 停止后 odom 位置约 `(0.6946, -0.0044, 0.3432)` 米。全局目标容差 0.5 米、MLS 栅格 0.2 米、局部位置容差 0.25 米；本测试验证接口与短程执行闭环，不代表厘米级定位精度。

关键日志：

```text
Loaded MLS from PLY '.../building_obstacles.ply' (200 200 cells @ 0.2 m).
FOUND_SOLUTION: 1 trajectory segment(s) (0.02 s)
FollowPath goal accepted.
Reached the goal!
Goal finished with status code: 4
All paths executed.
```

原碰撞中止原因：附近点云有 Z≈1.32 米的上方楼板，被 1.5 米高度阈值投影进二维代价地图。修正当前地面层障碍阈值为 Z=0.1～0.8 米后通过测试，碰撞检测未关闭。该过滤在世界坐标下工作，当前不支持高层/坡道；还需对变化高程采用相对高度过滤。

限制：只验证了一条短程路线；复杂障碍物绕行、动态障碍、上下楼和任务恢复均未据此宣称通过。地图加载时仍可见 PCL 共线样本提示，但本次规划和执行完成。
