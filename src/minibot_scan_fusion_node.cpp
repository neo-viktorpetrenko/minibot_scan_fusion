#include "minibot_scan_fusion/minibot_scan_fusion_node.hpp"

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/create_timer_ros.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <cmath>
#include <limits>
#include <algorithm>

namespace lidar_scan_fusion
{

static inline bool isFinitePositive(float v)
{
  return std::isfinite(v) && v > 0.0f;
}

LidarScanFusionNode::LidarScanFusionNode(const rclcpp::NodeOptions & options)
: Node("minibot_scan_fusion_node", options)
{
  // Topics/frames
  topic1_ = this->declare_parameter<std::string>("lidar1.topic", "/lidar1/scan");
  topic2_ = this->declare_parameter<std::string>("lidar2.topic", "/lidar2/scan");
  out_topic_ = this->declare_parameter<std::string>("output.topic", "/lidar_combined/scan");

  lidar1_frame_ = this->declare_parameter<std::string>("lidar1.frame", "laser_frame1");
  lidar2_frame_ = this->declare_parameter<std::string>("lidar2.frame", "laser_frame2");
  out_frame_    = this->declare_parameter<std::string>("output.frame", lidar1_frame_);

  // Sync + TF
  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 20);
  tf_cache_seconds_ = this->declare_parameter<double>("tf_cache_seconds", 2.0);

  // Lidar1 trim params
  p1_.enable_angle_exclusion = this->declare_parameter<bool>("lidar1.trim.enable_angle_exclusion", true);
  p1_.exclude_angle_min      = this->declare_parameter<double>("lidar1.trim.exclude_angle_min", -0.7);
  p1_.exclude_angle_max      = this->declare_parameter<double>("lidar1.trim.exclude_angle_max",  0.7);

  p1_.enable_range_clip      = this->declare_parameter<bool>("lidar1.trim.enable_range_clip", true);
  p1_.range_clip_min         = this->declare_parameter<double>("lidar1.trim.range_clip_min", 0.08);
  p1_.range_clip_max         = this->declare_parameter<double>("lidar1.trim.range_clip_max", 30.0);

  // Lidar2 trim params
  p2_.enable_angle_exclusion = this->declare_parameter<bool>("lidar2.trim.enable_angle_exclusion", true);
  p2_.exclude_angle_min      = this->declare_parameter<double>("lidar2.trim.exclude_angle_min", -0.7);
  p2_.exclude_angle_max      = this->declare_parameter<double>("lidar2.trim.exclude_angle_max",  0.7);

  p2_.enable_range_clip      = this->declare_parameter<bool>("lidar2.trim.enable_range_clip", true);
  p2_.range_clip_min         = this->declare_parameter<double>("lidar2.trim.range_clip_min", 0.08);
  p2_.range_clip_max         = this->declare_parameter<double>("lidar2.trim.range_clip_max", 30.0);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  pub_ = this->create_publisher<LaserScan>(out_topic_, qos);

  // TF buffer with node clock
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  // Use ROS timer interface for TF buffer internal timers
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  sub1_ = std::make_shared<message_filters::Subscriber<LaserScan>>(this, topic1_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sub2_ = std::make_shared<message_filters::Subscriber<LaserScan>>(this, topic2_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(sync_queue_size_), *sub1_, *sub2_);
  sync_->registerCallback(&LidarScanFusionNode::syncedCallback, this);

  RCLCPP_INFO(this->get_logger(),
    "minibot_scan_fusion_node started.\n"
    "  sub1: %s (%s)\n"
    "  sub2: %s (%s)\n"
    "  pub : %s (frame=%s)",
    topic1_.c_str(), lidar1_frame_.c_str(),
    topic2_.c_str(), lidar2_frame_.c_str(),
    out_topic_.c_str(), out_frame_.c_str());
}

bool LidarScanFusionNode::angleInExcludedSector(double a, double min_a, double max_a)
{
  // Works for normal (min<=max) and also wrap-around sectors if user sets min>max.
  if (min_a <= max_a) {
    return (a >= min_a && a <= max_a);
  }
  // Wrap-around: excluded if a >= min OR a <= max
  return (a >= min_a || a <= max_a);
}

void LidarScanFusionNode::trimScanInPlace(LaserScan & scan, const LidarTrimParams & p) const
{
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  const size_t n = scan.ranges.size();
  if (n == 0) return;

  const double a0 = scan.angle_min;
  const double da = scan.angle_increment;

  for (size_t i = 0; i < n; ++i) {
    const double a = a0 + static_cast<double>(i) * da;
    float & r = scan.ranges[i];

    // Range clip first
    if (p.enable_range_clip) {
      if (!std::isfinite(r) || r < static_cast<float>(p.range_clip_min) || r > static_cast<float>(p.range_clip_max)) {
        r = NaN;
        continue;
      }
    } else {
      if (!std::isfinite(r)) {
        r = NaN;
        continue;
      }
    }

    // Angle exclusion for self-filter
    if (p.enable_angle_exclusion) {
      if (angleInExcludedSector(a, p.exclude_angle_min, p.exclude_angle_max)) {
        r = NaN;
        continue;
      }
    }
  }
}

bool LidarScanFusionNode::getLidar2ToLidar1(tf2::Transform & out_T_l1_l2, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lk(tf_mutex_);

  // Use cached TF if "recent enough"
  if (has_cached_tf_) {
    const double age = std::abs((stamp - cached_stamp_).seconds());
    if (age <= tf_cache_seconds_) {
      out_T_l1_l2 = cached_T_l1_l2_;
      return true;
    }
  }

  // Lookup transform from lidar2 frame to lidar1 frame at scan time (or latest if exact not available)
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      lidar1_frame_,   // target
      lidar2_frame_,   // source
      stamp,
      rclcpp::Duration::from_seconds(0.05));
  } catch (const std::exception & e) {
    // Fall back to latest
    try {
      tf_msg = tf_buffer_->lookupTransform(lidar1_frame_, lidar2_frame_, tf2::TimePointZero);
    } catch (const std::exception & e2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (%s -> %s): %s",
        lidar2_frame_.c_str(), lidar1_frame_.c_str(), e2.what());
      return false;
    }
  }

  const auto & t = tf_msg.transform.translation;
  const auto & q = tf_msg.transform.rotation;

  tf2::Quaternion Q(q.x, q.y, q.z, q.w);
  tf2::Vector3    P(t.x, t.y, t.z);

  tf2::Transform T(Q, P);

  cached_T_l1_l2_ = T;
  cached_stamp_ = stamp;
  has_cached_tf_ = true;

  out_T_l1_l2 = T;
  return true;
}

void LidarScanFusionNode::fuseLidar2IntoLidar1Grid(
  LaserScan & out_scan_l1,
  const LaserScan & scan2_l2,
  const tf2::Transform & T_l1_l2,
  const LidarTrimParams & p2) const
{
  const size_t n_out = out_scan_l1.ranges.size();
  if (n_out == 0) return;

  const double out_a0 = out_scan_l1.angle_min;
  const double out_da = out_scan_l1.angle_increment;
  const double out_aN = out_a0 + (static_cast<double>(n_out) - 1.0) * out_da;

  const double a0_2 = scan2_l2.angle_min;
  const double da_2 = scan2_l2.angle_increment;
  const size_t n2 = scan2_l2.ranges.size();

  const float NaN = std::numeric_limits<float>::quiet_NaN();

  for (size_t i = 0; i < n2; ++i) {
    const double a2 = a0_2 + static_cast<double>(i) * da_2;

    // Angle exclusion for lidar2 BEFORE transforming (still valid for self-filtering its own mounting)
    if (p2.enable_angle_exclusion && angleInExcludedSector(a2, p2.exclude_angle_min, p2.exclude_angle_max)) {
      continue;
    }

    float r2 = scan2_l2.ranges[i];
    if (!std::isfinite(r2)) continue;

    if (p2.enable_range_clip) {
      if (r2 < static_cast<float>(p2.range_clip_min) || r2 > static_cast<float>(p2.range_clip_max)) {
        continue;
      }
    }

    // Convert beam endpoint into lidar2 frame (2D)
    const double x2 = static_cast<double>(r2) * std::cos(a2);
    const double y2 = static_cast<double>(r2) * std::sin(a2);

    // Transform into lidar1 frame
    const tf2::Vector3 p2_v(x2, y2, 0.0);
    const tf2::Vector3 p1_v = T_l1_l2 * p2_v;

    const double x1 = p1_v.x();
    const double y1 = p1_v.y();

    // Convert to polar in lidar1
    const double r1 = std::hypot(x1, y1);
    if (!(r1 > 0.0) || !std::isfinite(r1)) continue;

    const double a1 = std::atan2(y1, x1);

    // Check within lidar1 output scan angular bounds
    if (out_da > 0.0) {
      if (a1 < out_a0 || a1 > out_aN) continue;
    } else {
      // Unusual negative increments: handle conservatively
      if (a1 > out_a0 || a1 < out_aN) continue;
    }

    // Bin into lidar1 index
    const int idx = static_cast<int>(std::lround((a1 - out_a0) / out_da));
    if (idx < 0 || static_cast<size_t>(idx) >= n_out) continue;

    float & out_r = out_scan_l1.ranges[static_cast<size_t>(idx)];
    const float r1f = static_cast<float>(r1);

    bool wrote = false;

    if (!std::isfinite(out_r)) {
      out_r = r1f;
      wrote = true;
    } else if (r1f < out_r) {
      out_r = r1f;
      wrote = true;
    }

    if (wrote) {
      // If output has intensities, ensure it has correct size
      if (!out_scan_l1.intensities.empty()) {
        // If scan2 provides intensities, copy them; otherwise you can set to 0 or NaN.
        if (!scan2_l2.intensities.empty() && scan2_l2.intensities.size() == scan2_l2.ranges.size()) {
          out_scan_l1.intensities[static_cast<size_t>(idx)] = scan2_l2.intensities[i];
        }
        // else: leave lidar1 intensity as-is (or set to 0.0f if you prefer)
      }
    }

    // (Optional) intensities: keep as-is or set if you want.
    if (!out_scan_l1.intensities.empty()) {
      // You could also take max/min; here we keep lidar1's and ignore lidar2.
      (void)NaN;
    }
  }
}

void LidarScanFusionNode::syncedCallback(const LaserScanConstSharedPtr & scan1, const LaserScanConstSharedPtr & scan2)
{
  if (!scan1 || !scan2) return;
  if (scan1->ranges.empty() || scan2->ranges.empty()) return;

  // We will output in lidar1 scan grid
  LaserScan out = *scan1;

  if (out.intensities.empty() && !scan2->intensities.empty() && scan2->intensities.size() == scan2->ranges.size()) {
    out.intensities.resize(out.ranges.size(), std::numeric_limits<float>::quiet_NaN());
  }

  // Ensure output frame
  out.header.frame_id = out_frame_;

  // Trim lidar1 in-place
  trimScanInPlace(out, p1_);

  // Get TF lidar2 -> lidar1
  tf2::Transform T_l1_l2;
  if (!getLidar2ToLidar1(T_l1_l2, scan2->header.stamp)) {
    // Still publish trimmed lidar1 if TF missing (better than nothing)
    pub_->publish(out);
    return;
  }

  // Fuse lidar2 into lidar1 bins
  fuseLidar2IntoLidar1Grid(out, *scan2, T_l1_l2, p2_);

  pub_->publish(out);
}

}  // namespace lidar_scan_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lidar_scan_fusion::LidarScanFusionNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}