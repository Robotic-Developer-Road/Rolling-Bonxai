#include "rolling_bonxai_ros/rolling_bonxai_server.hpp"

#include <tf2/exceptions.h>

namespace RollingBonxai
{

RollingBonxaiServer::RollingBonxaiServer(
  const rclcpp::NodeOptions& options)
: rclcpp::Node("rolling_bonxai_server", options)
{
  loadParameters();
  initLogger();
  initTF();
  initMap();
  initSubscriber();
  initStatsPublishers();

  RCLCPP_INFO(get_logger(), "RollingBonxaiServer started");
}

void RollingBonxaiServer::loadParameters()
{
  server_params_.frame_id =
    declare_parameter<std::string>("frame_id", "map");

  server_params_.base_frame_id =
    declare_parameter<std::string>("base_frame_id", "base_footprint");

  server_params_.topic_in =
    declare_parameter<std::string>("topic_in", "/points");

  server_params_.cleanup_interval_sec =
    declare_parameter<double>("server.cleanup_interval_sec", 300.0);

  server_params_.enable_stats =
    declare_parameter<bool>("stats.enable_stats", true);

  server_params_.quick_stats =
    declare_parameter<bool>("stats.quick_stats", true);

  server_params_.publish_occupied_voxels =
    declare_parameter<bool>("stats.publish_occupied_voxels", true);

  server_params_.stats_publish_rate =
    declare_parameter<double>("stats.publish_rate", 1.0);

  // ---- RollingOccupancyMap parameters ----

  auto& mp = rolling_params_.map_params;

  mp.resolution =
    declare_parameter<double>("occupancy.resolution", 0.05);

  mp.occupancy_min_z =
    declare_parameter<double>("occupancy.occupancy_min_z", -100.0);

  mp.occupancy_max_z =
    declare_parameter<double>("occupancy.occupancy_max_z", 100.0);

  mp.occupancy_threshold =
    declare_parameter<double>("occupancy.occupancy_threshold", 0.5);

  mp.sensor_max_range =
    declare_parameter<double>("sensor_model.max_range", 30.0);

  mp.sensor_hit =
    declare_parameter<double>("sensor_model.hit", 0.7);

  mp.sensor_miss =
    declare_parameter<double>("sensor_model.miss", 0.4);

  mp.sensor_min =
    declare_parameter<double>("sensor_model.min", 0.12);

  mp.sensor_max =
    declare_parameter<double>("sensor_model.max", 0.97);

  rolling_params_.chunk_size =
    declare_parameter<double>(
      "rolling.coordinate_system.chunk_size", 5.0);

  rolling_params_.hysteresis_ratio =
    declare_parameter<double>(
      "rolling.transition_manager.hysteresis_ratio", 0.20);

  rolling_params_.loading_policy_name =
    declare_parameter<std::string>(
      "rolling.loading_policy.name", "neighbourhood");

  rolling_params_.nb_radius =
    declare_parameter<int>(
      "rolling.loading_policy.neighborhood.radius", 2);

  rolling_params_.planar_z_min =
    declare_parameter<int>(
      "rolling.loading_policy.planar.z_min", 1);

  rolling_params_.planar_z_max =
    declare_parameter<int>(
      "rolling.loading_policy.planar.z_max", 3);

  rolling_params_.planar_use_relative =
    declare_parameter<bool>(
      "rolling.loading_policy.planar.use_relative", false);

  rolling_params_.temporal_age_weightage =
    declare_parameter<double>(
      "rolling.loading_policy.temporal.age_weightage", 0.3);

  rolling_params_.temporal_max_age_s =
    declare_parameter<int>(
      "rolling.loading_policy.temporal.max_age_s", 300);

  rolling_params_.full_save_folder_path =
    declare_parameter<std::string>(
      "rolling.storage.full_save_folder_path", "");

  rolling_params_.asyncio.num_load_threads =
    static_cast<size_t>(
      declare_parameter<int>(
        "rolling.asyncio.num_load_threads", 3));

  rolling_params_.asyncio.num_save_threads =
    static_cast<size_t>(
      declare_parameter<int>(
        "rolling.asyncio.num_save_threads", 1));
}

void RollingBonxaiServer::initLogger()
{
  logger_ = std::make_shared<RosLogger>(get_logger());
}

void RollingBonxaiServer::initTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void RollingBonxaiServer::initMap()
{
  occupancy_map_ =
    std::make_unique<RollingBonxai::RollingOccupancyMap>(
      rolling_params_,
      logger_);
}

void RollingBonxaiServer::initSubscriber()
{
  pointcloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
      server_params_.topic_in,
      rclcpp::SensorDataQoS(),
      std::bind(
        &RollingBonxaiServer::pointcloudCallback,
        this,
        std::placeholders::_1));
}

void RollingBonxaiServer::initStatsPublishers()
{
  if (!server_params_.enable_stats) {
    return;
  }

  if (server_params_.publish_occupied_voxels) {
    voxel_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "occupied_voxels",
        rclcpp::QoS(1).transient_local());
  }

  chunk_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::ChunkStats>(
      "chunk_stats",
      rclcpp::QoS(10));

  const auto period =
    std::chrono::duration<double>(1.0 / server_params_.stats_publish_rate);

  stats_timer_ =
    create_wall_timer(
      period,
      std::bind(&RollingBonxaiServer::publishStats, this));
}

void RollingBonxaiServer::publishStats()
{
  const auto now_time = now();

  // ---------------- Occupied voxels ----------------
  if (voxel_pub_) {
    std::vector<Eigen::Vector3d> points;
    occupancy_map_->getOccupiedVoxels(points);

    if (!points.empty()) {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      cloud.reserve(points.size());

      for (const auto& p : points) {
        pcl::PointXYZRGB pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud.push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = server_params_.frame_id;
      msg.header.stamp = now_time;

      voxel_pub_->publish(msg);
    }
  }

  // ---------------- Chunk statistics ----------------
  rolling_bonxai_msgs::msg::ChunkStats stats_msg;
  stats_msg.header.stamp = now_time;
  stats_msg.header.frame_id = server_params_.frame_id;

  stats_msg.active_chunk_count =
    static_cast<uint32_t>(occupancy_map_->getActiveChunkCount());

  stats_msg.dirty_chunk_count =
    static_cast<uint32_t>(occupancy_map_->getDirtyChunkCount());

  stats_msg.clean_chunk_count =
    static_cast<uint32_t>(occupancy_map_->getCleanChunkCount());

  chunk_stats_pub_->publish(stats_msg);
}

void RollingBonxaiServer::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped =
      tf_buffer_->lookupTransform(
        server_params_.frame_id,
        msg->header.frame_id,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) {
    return;
  }

  Eigen::Vector3d sensor_origin(
    transform_stamped.transform.translation.x,
    transform_stamped.transform.translation.y,
    transform_stamped.transform.translation.z);

  Eigen::Vector3d sensor_linvel(0.0, 0.0, 0.0);

  Eigen::Quaterniond rotation(
    transform_stamped.transform.rotation.w,
    transform_stamped.transform.rotation.x,
    transform_stamped.transform.rotation.y,
    transform_stamped.transform.rotation.z);

  std::vector<Eigen::Vector3d> map_points;
  map_points.reserve(cloud->size());

  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z)) {
      continue;
    }

    Eigen::Vector3d pt_sensor(p.x, p.y, p.z);
    Eigen::Vector3d pt_map = rotation * pt_sensor + sensor_origin;

    if (pt_map.z() < rolling_params_.map_params.occupancy_min_z ||
        pt_map.z() > rolling_params_.map_params.occupancy_max_z) {
      continue;
    }

    map_points.push_back(pt_map);
  }

  if (!map_points.empty()) {
    occupancy_map_->insertPointCloud(
      map_points,
      sensor_origin,
      sensor_linvel,
      rolling_params_.map_params.sensor_max_range);
  }
}

} // namespace RollingBonxai

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RollingBonxai::RollingBonxaiServer)