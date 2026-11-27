# octo_planner

`octo_planner` and `jie_map_msgs` are the core ROS packages adapted from [`jie_3d_nav`](https://github.com/6-robot/jie_3d_nav). SCAN-Planner supplies live local obstacle avoidance in this workspace.

- `jie_path_node`: three-dimensional A* on `/jie/octomap`, publishing `/jie/planned_path`.
- `go2_path_follower`: D1 tracking of `/scan/local_path`, publishing `/cmd_vel_nav` and retaining `/jie/planned_path` for final-goal completion. An empty `local_path_topic` retains standalone global-path tracking.
- Go2 topics, frames and speeds: `go2_bringup/config/jie_navigation.yaml`.
- Go2 PCD map loading and goal/mode bridge: `go2_bringup`.

Start with `ros2 launch go2_bringup navigation.launch.py`. Upstream provenance and adaptation notes are in `UPSTREAM.md`.
