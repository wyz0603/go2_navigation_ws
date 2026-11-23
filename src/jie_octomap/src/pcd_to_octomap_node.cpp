#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
struct PointXYZ
{
  double x;
  double y;
  double z;
};

struct DownsampleKey
{
  long long x;
  long long y;
  long long z;
  bool operator==(const DownsampleKey & o) const { return x == o.x && y == o.y && z == o.z; }
};

struct DownsampleKeyHash
{
  std::size_t operator()(const DownsampleKey & k) const
  {
    std::size_t seed = std::hash<long long>{}(k.x);
    seed ^= std::hash<long long>{}(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<long long>{}(k.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

double readFloatField(const char * data, std::size_t offset, int size, char type)
{
  if (type != 'F') {
    throw std::runtime_error("PCD x/y/z fields must use floating point values");
  }
  if (size == 4) {
    float value;
    std::memcpy(&value, data + offset, sizeof(value));
    return static_cast<double>(value);
  }
  if (size == 8) {
    double value;
    std::memcpy(&value, data + offset, sizeof(value));
    return value;
  }
  throw std::runtime_error("PCD x/y/z fields must be 4 or 8 bytes");
}

bool readPcdFile(const std::string & path, std::vector<PointXYZ> & points)
{
  points.clear();
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }

  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::size_t point_count = 0;
  std::size_t width = 0;
  std::size_t height = 1;
  std::string data_mode;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream header(line);
    std::string key;
    header >> key;
    if (key == "FIELDS") {
      std::string value;
      while (header >> value) names.push_back(value);
    } else if (key == "SIZE") {
      int value;
      while (header >> value) sizes.push_back(value);
    } else if (key == "TYPE") {
      char value;
      while (header >> value) types.push_back(value);
    } else if (key == "COUNT") {
      int value;
      while (header >> value) counts.push_back(value);
    } else if (key == "WIDTH") {
      header >> width;
    } else if (key == "HEIGHT") {
      header >> height;
    } else if (key == "POINTS") {
      header >> point_count;
    } else if (key == "DATA") {
      header >> data_mode;
      break;
    }
  }

  if (point_count == 0) point_count = width * height;
  if (names.empty() || names.size() != sizes.size() || names.size() != types.size()) {
    return false;
  }
  if (counts.empty()) counts.assign(names.size(), 1);
  if (counts.size() != names.size()) {
    return false;
  }

  std::vector<std::size_t> offsets(names.size());
  std::size_t point_step = 0;
  std::size_t x_index = names.size();
  std::size_t y_index = names.size();
  std::size_t z_index = names.size();
  for (std::size_t i = 0; i < names.size(); ++i) {
    offsets[i] = point_step;
    point_step += static_cast<std::size_t>(sizes[i] * counts[i]);
    if (names[i] == "x") x_index = i;
    if (names[i] == "y") y_index = i;
    if (names[i] == "z") z_index = i;
  }
  if (x_index == names.size() || y_index == names.size() || z_index == names.size()) {
    return false;
  }

  points.reserve(point_count);
  if (data_mode == "binary") {
    std::vector<char> record(point_step);
    for (std::size_t i = 0; i < point_count; ++i) {
      if (!input.read(record.data(), static_cast<std::streamsize>(record.size()))) {
        break;
      }
      PointXYZ pt{
        readFloatField(record.data(), offsets[x_index], sizes[x_index], types[x_index]),
        readFloatField(record.data(), offsets[y_index], sizes[y_index], types[y_index]),
        readFloatField(record.data(), offsets[z_index], sizes[z_index], types[z_index])};
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        points.push_back(pt);
      }
    }
  } else if (data_mode == "ascii") {
    while (points.size() < point_count && std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;
      std::istringstream row(line);
      std::vector<double> values;
      double value;
      while (row >> value) values.push_back(value);
      if (values.size() < names.size()) continue;
      PointXYZ pt{values[x_index], values[y_index], values[z_index]};
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        points.push_back(pt);
      }
    }
  } else {
    return false;
  }
  return !points.empty();
}

std::vector<PointXYZ> voxelDownSample(const std::vector<PointXYZ> & pts, double voxel_size)
{
  if (voxel_size <= 0.0) return pts;
  struct Accum { double sx{0}, sy{0}, sz{0}; std::size_t n{0}; };
  std::unordered_map<DownsampleKey, Accum, DownsampleKeyHash> grid;
  grid.reserve(pts.size());
  for (const auto & p : pts) {
    DownsampleKey k{
      static_cast<long long>(std::floor(p.x / voxel_size)),
      static_cast<long long>(std::floor(p.y / voxel_size)),
      static_cast<long long>(std::floor(p.z / voxel_size))};
    auto & a = grid[k];
    a.sx += p.x; a.sy += p.y; a.sz += p.z; ++a.n;
  }
  std::vector<PointXYZ> out;
  out.reserve(grid.size());
  for (const auto & kv : grid) {
    const double inv = 1.0 / static_cast<double>(kv.second.n);
    out.push_back(PointXYZ{kv.second.sx * inv, kv.second.sy * inv, kv.second.sz * inv});
  }
  return out;
}
}  // namespace

class PcdToOctomapNode : public rclcpp::Node
{
public:
  PcdToOctomapNode()
  : Node("pcd_to_octomap")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("pcd_file_cmd_topic", "/pcd_file_cmd");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("voxel_downsample_m", 0.0);
    declare_parameter<int>("min_points_per_voxel", 3);
    declare_parameter<int>("min_cluster_voxels", 4);
    declare_parameter<double>("min_z", -1.0e9);
    declare_parameter<double>("max_z", 1.0e9);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    pcd_file_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("pcd_file_cmd_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&PcdToOctomapNode::onPcdFileCmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PcdToOctomapNode::publishMap, this));

    const auto pcd_file = get_parameter("pcd_file").as_string();
    if (!pcd_file.empty()) {
      loadPcd(pcd_file);
    } else {
      RCLCPP_INFO(get_logger(), "No initial pcd_file set. Waiting for /pcd_file_cmd.");
    }
  }

private:
  struct Key
  {
    unsigned int k[3];
    bool operator==(const Key & other) const
    {
      return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const
    {
      std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
      seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<unsigned int>{}(k(key));
      return seed;
    }
    static unsigned int k(const Key & key) { return key.k[2]; }
  };

  void onPcdFileCmd(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    loadPcd(msg->data);
  }

  void loadPcd(const std::string & pcd_file)
  {
    std::vector<PointXYZ> points;
    if (!readPcdFile(pcd_file, points)) {
      RCLCPP_ERROR(get_logger(), "Failed to read PCD file: %s", pcd_file.c_str());
      return;
    }

    const double voxel_downsample = get_parameter("voxel_downsample_m").as_double();
    if (voxel_downsample > 0.0) {
      points = voxelDownSample(points, voxel_downsample);
    }

    tree_ = std::make_shared<octomap::OcTree>(get_parameter("resolution").as_double());
    const int min_points_per_voxel =
      std::max(1, static_cast<int>(get_parameter("min_points_per_voxel").as_int()));
    const int min_cluster_voxels =
      std::max(1, static_cast<int>(get_parameter("min_cluster_voxels").as_int()));

    std::unordered_map<Key, std::size_t, KeyHash> voxel_counts;
    voxel_counts.reserve(points.size());
    for (const auto & point : points) {
      octomap::OcTreeKey raw_key;
      if (!tree_->coordToKeyChecked(
          static_cast<float>(point.x),
          static_cast<float>(point.y),
          static_cast<float>(point.z),
          raw_key))
      {
        continue;
      }
      const Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts[key];
    }

    std::unordered_set<Key, KeyHash> occupied_keys;
    occupied_keys.reserve(voxel_counts.size());
    for (const auto & entry : voxel_counts) {
      if (static_cast<int>(entry.second) >= min_points_per_voxel) {
        occupied_keys.insert(entry.first);
      }
    }

    std::size_t removed_cluster_voxels = 0;
    if (min_cluster_voxels > 1 && !occupied_keys.empty()) {
      std::unordered_set<Key, KeyHash> filtered_keys;
      filtered_keys.reserve(occupied_keys.size());
      std::unordered_set<Key, KeyHash> visited;
      visited.reserve(occupied_keys.size());

      for (const auto & seed : occupied_keys) {
        if (visited.find(seed) != visited.end()) {
          continue;
        }

        std::deque<Key> queue;
        std::vector<Key> cluster;
        queue.push_back(seed);
        visited.insert(seed);

        while (!queue.empty()) {
          const Key current = queue.front();
          queue.pop_front();
          cluster.push_back(current);

          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                  continue;
                }
                const int nx = static_cast<int>(current.k[0]) + dx;
                const int ny = static_cast<int>(current.k[1]) + dy;
                const int nz = static_cast<int>(current.k[2]) + dz;
                if (nx < 0 || ny < 0 || nz < 0) {
                  continue;
                }
                const Key neighbor{{
                  static_cast<unsigned int>(nx),
                  static_cast<unsigned int>(ny),
                  static_cast<unsigned int>(nz)}};
                if (occupied_keys.find(neighbor) == occupied_keys.end() ||
                    visited.find(neighbor) != visited.end())
                {
                  continue;
                }
                visited.insert(neighbor);
                queue.push_back(neighbor);
              }
            }
          }
        }

        if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
          filtered_keys.insert(cluster.begin(), cluster.end());
        } else {
          removed_cluster_voxels += cluster.size();
        }
      }

      occupied_keys.swap(filtered_keys);
    }

    for (const auto & key : occupied_keys) {
      octomap::OcTreeKey octo_key;
      octo_key.k[0] = static_cast<octomap::key_type>(key.k[0]);
      octo_key.k[1] = static_cast<octomap::key_type>(key.k[1]);
      octo_key.k[2] = static_cast<octomap::key_type>(key.k[2]);
      tree_->updateNode(tree_->keyToCoord(octo_key), true);
    }
    tree_->updateInnerOccupancy();
    current_pcd_file_ = pcd_file;

    RCLCPP_INFO(
      get_logger(),
      "Loaded PCD file: %s, source_points=%zu, candidate_voxels=%zu, kept_voxels=%zu, "
      "removed_cluster_voxels=%zu, occupied_voxels=%zu, resolution=%.3f, voxel_downsample_m=%.3f, "
      "min_points_per_voxel=%d, min_cluster_voxels=%d",
      pcd_file.c_str(), points.size(), voxel_counts.size(), occupied_keys.size(),
      removed_cluster_voxels, tree_->calcNumNodes(), get_parameter("resolution").as_double(),
      voxel_downsample, min_points_per_voxel, min_cluster_voxels);
    publishMap();
  }

  void publishMap()
  {
    if (!tree_) {
      return;
    }

    octomap_msgs::msg::Octomap map_msg;
    map_msg.header.frame_id = get_parameter("frame_id").as_string();
    map_msg.header.stamp = now();

    if (!octomap_msgs::fullMapToMsg(*tree_, map_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to serialize OctoMap.");
      return;
    }

    octomap_pub_->publish(map_msg);
  }

  std::shared_ptr<octomap::OcTree> tree_;
  std::string current_pcd_file_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcd_file_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
