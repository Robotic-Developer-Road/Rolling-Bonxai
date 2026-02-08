#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rolling_bonxai/rolling_occupancy_map.hpp"

// Logger interfaces
#include "rolling_bonxai/logger.hpp"
#include "rolling_bonxai_ros/ros_logger.hpp"

// Messages
#include "rolling_bonxai_msgs/msg/chunk_stats.hpp"

namespace RollingBonxai
{

struct ServerParams
{
  std::string frame_id;
  std::string base_frame_id;
  std::string topic_in;

  double cleanup_interval_sec{300.0};

  bool enable_stats{true};
  bool quick_stats{true};
  bool publish_occupied_voxels{true};
  double stats_publish_rate{1.0};
};

class RollingBonxaiServer : public rclcpp::Node
{
public:
  explicit RollingBonxaiServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Load all ROS parameters into server and map parameter structs
   */
  void loadParameters();

  /**
   * @brief Initialize framework-agnostic logger backed by rclcpp::Logger
   */
  void initLogger();

  /**
   * @brief Initialize TF2 buffer and listener
   */
  void initTF();

  /**
   * @brief Construct the rolling occupancy map
   */
  void initMap();

  /**
   * @brief Initialize PointCloud2 subscriber
   */
  void initSubscriber();

  /**
   * @brief Initialize statistics publishers and timer
   */
  void initStatsPublishers();

  /**
   * @brief Periodic statistics publishing callback
   *
   * Publishes:
   *  - Occupied voxel PointCloud2 (red points, if enabled)
   *  - ChunkStats message (active / dirty / clean chunks)
   *
   * @complexity
   *  - Voxels: O(N) where N = occupied voxels
   *  - Chunks: O(M) where M = active chunks
   */
  void publishStats();

  /**
   * @brief Handle incoming sensor point clouds
   * @param msg Incoming PointCloud2 message
   */
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  /// Server-level parameters
  ServerParams server_params_;

  /// Canonical rolling map parameters
  RollingOccupancyMap::AllParameters rolling_params_;

  /// Framework-agnostic logger
  std::shared_ptr<Logger> logger_;

  /// TF infrastructure
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// Input point cloud subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Rolling occupancy map
  std::unique_ptr<RollingBonxai::RollingOccupancyMap> occupancy_map_;

  /// Stats publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<rolling_bonxai_msgs::msg::ChunkStats>::SharedPtr chunk_stats_pub_;

  /// Stats timer
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

} // namespace RollingBonxai
