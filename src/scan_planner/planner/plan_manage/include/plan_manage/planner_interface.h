/**
 * @file      planner_interface.h
 * @brief     SCAN-Planner standalone trajectory optimization interface.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-07-23 20:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This header exposes path, obstacle, map initialization, and local trajectory planning APIs.
 */

#ifndef PLAN_MANAGE_PLANNER_INTERFACE_H_
#define PLAN_MANAGE_PLANNER_INTERFACE_H_

#include <chrono>
#include <memory>
#include <vector>

#include "Bspline.h"
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>

namespace scan_planner
{
struct PathPoint
{
    float x;
    float y;
    float z;
    float v;
    float theta;
};

struct ObstacleInfo
{
    float x;
    float y;
    float z = 0.2f;
};

class PlannerInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlannerInterface();
    ~PlannerInterface();

    void initParam(double max_vel, double max_acc, double max_jerk);
    void initEsdfMap(double x_size, double y_size, double z_size,
                     double resolution, Eigen::Vector3d org,
                     double inflate_values, double cylinder_offset = 0.35);
    void setPathPoint(std::vector<PathPoint> &plan_traj);
    void setObstacles(std::vector<ObstacleInfo> &obstacle);
    void makePlan();
    void setGridMapPos(PathPoint &cur_pose);
    void setVelocity(const Eigen::Vector3d &velocity);
    void getObstacles(std::vector<Eigen::Vector3d> &obstacle) const;
    void getAStarPath(
        std::vector<std::vector<Eigen::Vector3d>> &a_star_path);
    void getLocalPlanTrajResults(
        std::vector<PathPoint> &plan_traj_results);
    bool trajectoryIsCollisionFree(const std::vector<PathPoint> &trajectory) const;
    bool rotationIsCollisionFree(const PathPoint &position, double start_yaw, double end_yaw) const;
    static std::vector<PathPoint> trajectoryPrefix(const std::vector<PathPoint> &trajectory, double max_length);
    void getTraj();

private:
    bool reboundReplan(
        Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
        Eigen::Vector3d local_target_vel,
        std::vector<Eigen::Vector3d> point_set);
    void applyLinearZReference(
        std::vector<Eigen::Vector3d> &points, double start_z,
        double target_z);
    void updateTrajInfo(const UniformBspline &position_traj);
    void reparamBspline(
        UniformBspline &bspline,
        std::vector<Eigen::Vector3d> &start_end_derivative,
        double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
        double &time_inc);
    bool refineTrajAlgo(
        UniformBspline &traj,
        std::vector<Eigen::Vector3d> &start_end_derivative,
        double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    PlanParameters pp_;
    LocalTrajData local_data_;
    std::shared_ptr<plan_env::GridMap> grid_map_;
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;
    int continous_failures_count_{0};
    std::vector<PathPoint> _global_plan_traj_;
    std::vector<PathPoint> _plan_traj_results_;
    std::vector<Eigen::Vector3d> global_cloud_;
    Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes_;
    PathPoint cur_pose_{};
};

}  // namespace scan_planner


#endif  // PLAN_MANAGE_PLANNER_INTERFACE_H_
