#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <mutex>

namespace lidar_scan_fusion
{

struct LidarTrimParams
{
  // Angular sector to remove (robot self-occlusion), radians.
  // If enabled and angle_in_exclusion(theta) => range set to NaN.
  bool enable_angle_exclusion{true};
  double exclude_angle_min{-0.5};
  double exclude_angle_max{0.5};

  // Range clipping (meters). If enabled, ranges outside [min,max] are ignored.
  bool enable_range_clip{true};
  double range_clip_min{0.05};
  double range_clip_max{30.0};
};

class LidarScanFusionNode : public rclcpp::Node
{
public:
  explicit LidarScanFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using LaserScan = sensor_msgs::msg::LaserScan;
  using LaserScanConstSharedPtr = LaserScan::ConstSharedPtr;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;

  void syncedCallback(const LaserScanConstSharedPtr & scan1, const LaserScanConstSharedPtr & scan2);

  void trimScanInPlace(LaserScan & scan, const LidarTrimParams & p) const;

  bool getLidar2ToLidar1(tf2::Transform & out_T_l1_l2, const rclcpp::Time & stamp);

  // Fuse lidar2 into lidar1 scan grid (lidar1 defines output angle bins).
  void fuseLidar2IntoLidar1Grid(
    LaserScan & out_scan_l1,
    const LaserScan & scan2_l2,
    const tf2::Transform & T_l1_l2,
    const LidarTrimParams & p2) const;

  static bool angleInExcludedSector(double a, double min_a, double max_a);

  // Params
  std::string topic1_;
  std::string topic2_;
  std::string out_topic_;

  std::string lidar1_frame_;
  std::string lidar2_frame_;
  std::string out_frame_; // usually lidar1_frame_

  LidarTrimParams p1_;
  LidarTrimParams p2_;

  int sync_queue_size_{20};
  double tf_cache_seconds_{2.0};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached transform lidar2->lidar1
  std::mutex tf_mutex_;
  tf2::Transform cached_T_l1_l2_;
  rclcpp::Time cached_stamp_;
  bool has_cached_tf_{false};

  // Subscribers + sync
  std::shared_ptr<message_filters::Subscriber<LaserScan>> sub1_;
  std::shared_ptr<message_filters::Subscriber<LaserScan>> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
};

}  // namespace lidar_scan_fusion