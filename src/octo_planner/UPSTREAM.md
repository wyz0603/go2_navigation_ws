# Upstream provenance

The A* planner and path tracking controller are adapted from `6-robot/jie_3d_nav`, commit `995a3a5`.

The upstream package declares the MIT license. Go2 integration keeps the upstream planner and controller algorithms, removes the D1 launch stack, and adds Go2-specific launch, map loading, and command selection in `go2_bringup`.
