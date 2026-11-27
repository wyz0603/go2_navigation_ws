#include <gtest/gtest.h>
#include "plan_env/grid_map.h"
#include "planner_interface.h"
#include "path_searching/dyn_a_star.h"

TEST(ScanGrid, TracksRobotFloorAndKeepsObstacleHeight)
{
  plan_env::GridMapConfig config;
  config.resolution = 0.1;
  config.local_map_size = Eigen::Vector3d(4.0, 4.0, 4.0);
  config.follow_robot_z = true;
  plan_env::GridMap grid(config);
  const Eigen::Vector3d robot(0.0, 0.0, 6.35);
  const Eigen::Vector3d obstacle(0.4, 0.0, 6.5);
  grid.updateLocalMap(robot, {obstacle});
  EXPECT_EQ(grid.queryRaw(obstacle), plan_env::GridMap::CellState::kOccupied);
  EXPECT_EQ(grid.queryRaw(Eigen::Vector3d(0.4, 0.0, 0.2)),
            plan_env::GridMap::CellState::kOutOfMap);
}

TEST(ScanPlanner, EmptyOrShortPathDoesNotPlan)
{
  scan_planner::PlannerInterface planner;
  std::vector<scan_planner::PathPoint> result;
  planner.makePlan();
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
  EXPECT_FALSE(planner.trajectoryIsCollisionFree(result));
  std::vector<scan_planner::PathPoint> short_path(3);
  planner.setPathPoint(short_path);
  planner.makePlan();
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
  short_path.resize(4);
  planner.setPathPoint(short_path);
  planner.makePlan();
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
}

TEST(ScanPlanner, TrajectoryPrefixPreservesHeightAndCapsLength)
{
  std::vector<scan_planner::PathPoint> input(3);
  input[0].x = 0.0; input[0].y = 0.0; input[0].z = 3.0;
  input[1].x = 1.0; input[1].y = 0.0; input[1].z = 3.0;
  input[2].x = 2.0; input[2].y = 0.0; input[2].z = 4.0;
  auto result = scan_planner::PlannerInterface::trajectoryPrefix(input, 1.5);
  ASSERT_EQ(result.size(), 3u);
  EXPECT_NEAR(result.back().x, 1.0 + 0.5 / std::sqrt(2.0), 1e-5);
  EXPECT_NEAR(result.back().z, 3.0 + 0.5 / std::sqrt(2.0), 1e-5);
  EXPECT_TRUE(scan_planner::PlannerInterface::trajectoryPrefix(input, 0.0).empty());
}

TEST(ScanAStar, SameVoxelEndpointsProduceUsablePath)
{
  plan_env::GridMapConfig config;
  config.resolution = 0.1;
  config.local_map_size = Eigen::Vector3d(4.0, 4.0, 4.0);
  config.follow_robot_z = true;
  auto grid = std::make_shared<plan_env::GridMap>(config);
  grid->updateLocalMap(Eigen::Vector3d(0, 0, 0.35), {});
  AStar search;
  search.initGridMap(grid, Eigen::Vector3i(20, 20, 20));
  EXPECT_EQ(search.AstarSearch(0.1, Eigen::Vector3d(0, 0, 0.35),
                               Eigen::Vector3d(0.01, 0, 0.35)), ASTAR_RET::SUCCESS);
  EXPECT_GE(search.getPath().size(), 2u);
}

TEST(ScanAStar, RejectsOutsideLocalMap)
{
  plan_env::GridMapConfig config;
  config.local_map_size = Eigen::Vector3d(2.0, 2.0, 2.0);
  config.follow_robot_z = true;
  auto grid = std::make_shared<plan_env::GridMap>(config);
  grid->updateLocalMap(Eigen::Vector3d(0, 0, 0.35), {});
  AStar search;
  search.initGridMap(grid, Eigen::Vector3i(20, 20, 20));
  EXPECT_EQ(search.AstarSearch(0.1, Eigen::Vector3d(3, 0, 0.35),
                               Eigen::Vector3d(3.2, 0, 0.35)), ASTAR_RET::INIT_ERR);
}

TEST(ScanPlanner, OccupiedStartStopsWithoutOptimizing)
{
  scan_planner::PlannerInterface planner;
  planner.initParam(0.5, 1.0, 2.0);
  planner.initEsdfMap(6, 6, 4, 0.1, Eigen::Vector3d::Zero(), 0.25, 0.15);
  scan_planner::PathPoint robot{};
  robot.z = 0.35;
  planner.setGridMapPos(robot);
  std::vector<scan_planner::ObstacleInfo> obstacles{{0.0f, 0.0f, 0.35f}};
  planner.setObstacles(obstacles);
  std::vector<scan_planner::PathPoint> path(21, robot);
  for (size_t i = 0; i < path.size(); ++i) path[i].x = 0.1 * i;
  planner.setPathPoint(path);
  planner.makePlan();
  std::vector<scan_planner::PathPoint> result;
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
  EXPECT_FALSE(planner.trajectoryIsCollisionFree(path));
}

TEST(ScanPlanner, TerminalCollisionChecksUseLatestObservation)
{
  scan_planner::PlannerInterface planner;
  planner.initEsdfMap(6, 6, 4, 0.1, Eigen::Vector3d::Zero(), 0.25, 0.15);
  scan_planner::PathPoint robot{};
  robot.z = 0.35;
  planner.setGridMapPos(robot);
  std::vector<scan_planner::ObstacleInfo> obstacles{{0.0f, 0.0f, 0.35f}};
  planner.setObstacles(obstacles);
  EXPECT_FALSE(planner.rotationIsCollisionFree(robot, 0.0, 0.5));
  std::vector<scan_planner::PathPoint> stationary(2, robot);
  EXPECT_FALSE(planner.trajectoryIsCollisionFree(stationary));
  obstacles.clear();
  planner.setObstacles(obstacles);
  EXPECT_TRUE(planner.rotationIsCollisionFree(robot, 0.0, 0.5));
  EXPECT_TRUE(planner.trajectoryIsCollisionFree(stationary));
}
