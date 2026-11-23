# octo_planner

`octo_planner` and `jie_map_msgs` are the core ROS packages adapted from [`jie_3d_nav`](https://github.com/6-robot/jie_3d_nav). They replace the former PCT/SCAN planning packages in this Go2 workspace.

- `jie_path_node`: three-dimensional A* on `/jie/octomap`, publishing `/jie/planned_path`.
- `go2_path_follower`: D1 path tracking, publishing navigation commands to `/cmd_vel_nav`.
- Go2 topics, frames and speeds: `go2_bringup/config/jie_navigation.yaml`.
- Go2 PCD map loading and goal/mode bridge: `go2_bringup`.

Start with `ros2 launch go2_bringup navigation.launch.py`. Upstream provenance and adaptation notes are in `UPSTREAM.md`.
