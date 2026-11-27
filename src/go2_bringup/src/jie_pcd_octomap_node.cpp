#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
struct PointXYZ
{
  float x;
  float y;
  float z;
};

struct VoxelKey
{
  unsigned int x;
  unsigned int y;
  unsigned int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<unsigned int>{}(key.x);
    seed ^= std::hash<unsigned int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<unsigned int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

float readFloatField(const char * data, std::size_t offset, int size, char type)
{
  if (type != 'F') {
    throw std::runtime_error("PCD x/y/z fields must use floating point values");
  }
  if (size == 4) {
    float value;
    std::memcpy(&value, data + offset, sizeof(value));
    return value;
  }
  if (size == 8) {
    double value;
    std::memcpy(&value, data + offset, sizeof(value));
    return static_cast<float>(value);
  }
  throw std::runtime_error("PCD x/y/z fields must be 4 or 8 bytes");
}

std::vector<PointXYZ> readPcd(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open PCD file: " + path);
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
    throw std::runtime_error("PCD header has incomplete FIELDS/SIZE/TYPE entries");
  }
  if (counts.empty()) counts.assign(names.size(), 1);
  if (counts.size() != names.size()) {
    throw std::runtime_error("PCD COUNT entries do not match FIELDS");
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
  if (x_index == names.size() || y_index == names.size() || z_index == names.size() ||
      counts[x_index] != 1 || counts[y_index] != 1 || counts[z_index] != 1) {
    throw std::runtime_error("PCD must contain scalar x, y, and z fields");
  }
  if (data_mode != "binary" && data_mode != "ascii") {
    throw std::runtime_error("PCD DATA mode must be binary or ascii");
  }

  std::vector<PointXYZ> points;
  points.reserve(point_count);
  if (data_mode == "binary") {
    std::vector<char> record(point_step);
    for (std::size_t i = 0; i < point_count; ++i) {
      if (!input.read(record.data(), static_cast<std::streamsize>(record.size()))) {
        throw std::runtime_error("PCD binary data ended before POINTS count");
      }
      PointXYZ point{
        readFloatField(record.data(), offsets[x_index], sizes[x_index], types[x_index]),
        readFloatField(record.data(), offsets[y_index], sizes[y_index], types[y_index]),
        readFloatField(record.data(), offsets[z_index], sizes[z_index], types[z_index])};
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }
  } else {
    while (points.size() < point_count && std::getline(input, line)) {
      if (line.empty()) continue;
      std::istringstream row(line);
      std::vector<double> values;
      double value;
      while (row >> value) values.push_back(value);
      if (values.size() < names.size()) {
        throw std::runtime_error("PCD ASCII row has fewer values than FIELDS");
      }
      PointXYZ point{
        static_cast<float>(values[x_index]),
        static_cast<float>(values[y_index]),
        static_cast<float>(values[z_index])};
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }
    if (points.size() != point_count) {
      throw std::runtime_error("PCD ASCII data ended before POINTS count");
    }
  }
  return points;
}
}  // namespace

class JiePcdOctomapNode : public rclcpp::Node
{
public:
  JiePcdOctomapNode()
  : Node("jie_pcd_octomap")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<std::string>("octomap_topic", "/jie/octomap");
    declare_parameter<std::string>("occupied_marker_topic", "/jie/octomap_occupied_markers");
    declare_parameter<std::string>("occupied_levels_topic", "/jie/octomap_occupied_levels");
    declare_parameter<std::string>("pointcloud_topic", "/jie/map_points");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<int>("min_points_per_voxel", 1);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("occupied_marker_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    occupied_levels_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      get_parameter("occupied_levels_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());

    const auto pcd_file = get_parameter("pcd_file").as_string();
    const double resolution = get_parameter("resolution").as_double();
    if (pcd_file.empty() || !std::isfinite(resolution) || resolution <= 0.0) {
      throw std::runtime_error("pcd_file and a positive resolution are required");
    }

    const auto points = readPcd(pcd_file);
    tree_ = std::make_unique<octomap::OcTree>(resolution);
    const int min_points = std::max(1, static_cast<int>(
      get_parameter("min_points_per_voxel").as_int()));
    std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxel_counts;
    voxel_counts.reserve(points.size());
    for (const auto & point : points) {
      octomap::OcTreeKey raw_key;
      if (!tree_->coordToKeyChecked(point.x, point.y, point.z, raw_key)) continue;
      ++voxel_counts[VoxelKey{raw_key.k[0], raw_key.k[1], raw_key.k[2]}];
    }

    std::size_t occupied_voxels = 0;
    for (const auto & entry : voxel_counts) {
      if (static_cast<int>(entry.second) < min_points) continue;
      octomap::OcTreeKey key;
      key.k[0] = static_cast<octomap::key_type>(entry.first.x);
      key.k[1] = static_cast<octomap::key_type>(entry.first.y);
      key.k[2] = static_cast<octomap::key_type>(entry.first.z);
      tree_->updateNode(tree_->keyToCoord(key), true);
      ++occupied_voxels;
    }
    tree_->updateInnerOccupancy();
    publishMap(points);
    RCLCPP_INFO(get_logger(),
      "Loaded PCD into Go2 OctoMap: source_points=%zu occupied_voxels=%zu resolution=%.3f",
      points.size(), occupied_voxels, resolution);
  }

private:
  void publishMap(const std::vector<PointXYZ> & points)
  {
    const auto stamp = now();
    const auto frame_id = get_parameter("frame_id").as_string();

    octomap_msgs::msg::Octomap map;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map)) {
      throw std::runtime_error("failed to serialize the OctoMap");
    }
    map.header.stamp = stamp;
    map.header.frame_id = frame_id;
    octomap_pub_->publish(map);

    visualization_msgs::msg::Marker marker;
    marker.header = map.header;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double res = tree_->getResolution();
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.85F;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        continue;
      }
      geometry_msgs::msg::Point pt;
      pt.x = it.getX();
      pt.y = it.getY();
      pt.z = it.getZ();
      marker.points.push_back(pt);
    }
    occupied_marker_pub_->publish(marker);

    // Also publish floor-sliced MarkerArray so RViz users can hide roof/upper floors with 1 click
    const char * level_names[5] = {
      "1_floor_1_0m",
      "2_floor_2_1p5m",
      "3_floor_3_3p0m",
      "4_floor_4_4p5m",
      "5_roof_5p8m"
    };
    visualization_msgs::msg::MarkerArray level_array;
    level_array.markers.resize(5);
    for (int i = 0; i < 5; ++i) {
      auto & m = level_array.markers[i];
      m.header = map.header;
      m.ns = level_names[i];
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = res;
      m.scale.y = res;
      m.scale.z = res;
      m.color.r = 0.95F;
      m.color.g = 0.45F;
      m.color.b = 0.15F;
      m.color.a = 0.85F;
    }
    for (const auto & pt : marker.points) {
      int lvl = 4;
      if (pt.z < 1.35) {
        lvl = 0;
      } else if (pt.z < 2.85) {
        lvl = 1;
      } else if (pt.z < 4.30) {
        lvl = 2;
      } else if (pt.z < 5.75) {
        lvl = 3;
      }
      level_array.markers[lvl].points.push_back(pt);
    }
    occupied_levels_pub_->publish(level_array);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = map.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point.x; *y = point.y; *z = point.z;
      ++x; ++y; ++z;
    }
    pointcloud_pub_->publish(cloud);
  }

  std::unique_ptr<octomap::OcTree> tree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_levels_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JiePcdOctomapNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("jie_pcd_octomap"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
