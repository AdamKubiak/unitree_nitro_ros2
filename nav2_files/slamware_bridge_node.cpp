
/**
* slamware_bridge_node.cpp - ROS2 bridge for Slamtec Mapper M2M2
*
* Connects to the Mapper via Slamware SDK and publishes:
*   /scan         (sensor_msgs/LaserScan)      - lidar data at ~10Hz
*   /map          (nav_msgs/OccupancyGrid)      - occupancy map at ~0.5Hz
*   /mapper/pose  (geometry_msgs/PoseStamped)   - robot pose at ~10Hz
*   /tf           map -> odom transform         - used by Nav2 TF chain
*
* Navigation is handled externally by Nav2.
* odom -> base_link TF is provided by odom_publisher.py (Go1 high_state).
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace rpos::robot_platforms;
using namespace rpos::features::location_provider;

class SlamwareBridgeNode : public rclcpp::Node
{
public:
SlamwareBridgeNode()
: Node("slamware_bridge")
{
// ---- Parameters ----
declare_parameter("mapper_ip",   "192.168.11.1");
declare_parameter("mapper_port", 1445);
declare_parameter("scan_topic",  "/scan");
declare_parameter("map_topic",   "/map");
declare_parameter("scan_rate",   10.0);
declare_parameter("map_rate",    0.5);
declare_parameter("scan_frame_id",  "laser_frame");
declare_parameter("map_frame_id",   "map");
declare_parameter("base_frame_id",  "base_link");
declare_parameter("odom_frame_id",  "odom");
declare_parameter("range_min",  0.1);
declare_parameter("range_max",  40.0);
// laser_frame offset from base_link (meters)
declare_parameter("laser_x_offset", 0.0);
declare_parameter("laser_y_offset", 0.0);
declare_parameter("laser_z_offset", 0.0);

mapper_ip_    = get_parameter("mapper_ip").as_string();
mapper_port_  = get_parameter("mapper_port").as_int();
scan_frame_id_ = get_parameter("scan_frame_id").as_string();
map_frame_id_  = get_parameter("map_frame_id").as_string();
base_frame_id_ = get_parameter("base_frame_id").as_string();
odom_frame_id_ = get_parameter("odom_frame_id").as_string();
range_min_ = get_parameter("range_min").as_double();
range_max_ = get_parameter("range_max").as_double();

// ---- Publishers ----
auto scan_qos = rclcpp::QoS(10).reliable();
scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
  get_parameter("scan_topic").as_string(), scan_qos);
pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("mapper/pose", scan_qos);

// Map uses transient_local so late subscribers (Nav2 costmaps) receive it
auto map_qos = rclcpp::QoS(1).transient_local().reliable();
map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
  get_parameter("map_topic").as_string(), map_qos);

// ---- TF broadcasters ----
tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

// Publish static TF: base_link -> laser_frame
publishStaticLaserTF();

// ---- Connect to Mapper ----
RCLCPP_INFO(get_logger(), "Connecting to Slamware Mapper at %s:%ld ...",
  mapper_ip_.c_str(), mapper_port_);

try {
  platform_ = SlamwareCorePlatform::connect(mapper_ip_, (int)mapper_port_, 10000);
  connected_ = true;

  std::string sdkVer, sdpVer;
  try { sdkVer = platform_.getSDKVersion(); } catch (...) { sdkVer = "?"; }
  try { sdpVer = platform_.getSDPVersion(); } catch (...) { sdpVer = "?"; }

  RCLCPP_INFO(get_logger(), "Connected! SDK=%s SDP=%s",
    sdkVer.c_str(), sdpVer.c_str());
} catch (const std::exception& e) {
  RCLCPP_ERROR(get_logger(), "Failed to connect to Mapper: %s", e.what());
  RCLCPP_ERROR(get_logger(), "Make sure you're on the Mapper's WiFi network.");
  return;
}

// ---- Timers ----
double scan_rate = get_parameter("scan_rate").as_double();
double map_rate  = get_parameter("map_rate").as_double();

scan_timer_ = create_wall_timer(
  std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate)),
  std::bind(&SlamwareBridgeNode::scanTimerCallback, this));

map_timer_ = create_wall_timer(
  std::chrono::milliseconds(static_cast<int>(1000.0 / map_rate)),
  std::bind(&SlamwareBridgeNode::mapTimerCallback, this));

RCLCPP_INFO(get_logger(), "Slamware Bridge ready — scan %.0fHz, map %.1fHz",
  scan_rate, map_rate);
}

~SlamwareBridgeNode()
{
if (connected_) {
try { platform_.disconnect(); } catch (...) {}
}
}

private:
// ================================================================
// Scan + Pose timer (high frequency, ~10Hz)
// ================================================================
void scanTimerCallback()
{
if (!connected_) return;
auto stamp = this->now();

try {
publishLaserScan(stamp);
} catch (const std::exception& e) {
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
"Laser scan error: %s", e.what());
}

try {
publishPoseAndTF(stamp);
} catch (const std::exception& e) {
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
"Pose error: %s", e.what());
}
}

void publishLaserScan(const rclcpp::Time& stamp)
{
auto sdkScan = platform_.getLaserScan();
const auto& points = sdkScan.getLaserPoints();
if (points.empty()) return;

auto sorted = points;
std::sort(sorted.begin(), sorted.end(),
[](const rpos::core::LaserPoint& a, const rpos::core::LaserPoint& b) {
return a.angle() < b.angle();
});

sensor_msgs::msg::LaserScan msg;
msg.header.stamp = stamp;
msg.header.frame_id = scan_frame_id_;

msg.angle_min = sorted.front().angle();
msg.angle_max = sorted.back().angle();
size_t n = sorted.size();
msg.angle_increment = (n > 1)
? (msg.angle_max - msg.angle_min) / static_cast<float>(n - 1)
: 0.0f;
msg.time_increment = 0.0f;
msg.scan_time = 0.1f;
msg.range_min = static_cast<float>(range_min_);
msg.range_max = static_cast<float>(range_max_);

msg.ranges.resize(n);
msg.intensities.resize(n);

for (size_t i = 0; i < n; ++i) {
if (sorted[i].valid() && sorted[i].distance() > 0.0f) {
msg.ranges[i] = sorted[i].distance();
msg.intensities[i] = 1.0f;
} else {
msg.ranges[i] = std::numeric_limits<float>::infinity();
msg.intensities[i] = 0.0f;
}
}

scan_pub_->publish(msg);
}

void publishPoseAndTF(const rclcpp::Time& stamp)
{
auto pose = platform_.getPose();

tf2::Quaternion q;
q.setRPY(0.0, 0.0, pose.yaw());

geometry_msgs::msg::PoseStamped pose_msg;
pose_msg.header.stamp = stamp;
pose_msg.header.frame_id = map_frame_id_;
pose_msg.pose.position.x = pose.x();
pose_msg.pose.position.y = pose.y();
pose_msg.pose.position.z = 0.0;
pose_msg.pose.orientation.x = q.x();
pose_msg.pose.orientation.y = q.y();
pose_msg.pose.orientation.z = q.z();
pose_msg.pose.orientation.w = q.w();
pose_pub_->publish(pose_msg);

// Publish map -> odom TF.
// odom -> base_link is provided by odom_publisher.py from Go1 high_state.
// Together they form the required Nav2 chain: map -> odom -> base_link.
geometry_msgs::msg::TransformStamped tf_msg;
tf_msg.header.stamp = stamp;
tf_msg.header.frame_id = map_frame_id_;
tf_msg.child_frame_id = odom_frame_id_;
tf_msg.transform.translation.x = pose.x();
tf_msg.transform.translation.y = pose.y();
tf_msg.transform.translation.z = 0.0;
tf_msg.transform.rotation = pose_msg.pose.orientation;
tf_broadcaster_->sendTransform(tf_msg);
}

// ================================================================
// Map timer (low frequency, ~0.5Hz)
// ================================================================
void mapTimerCallback()
{
if (!connected_) return;

try {
MapType mapType = MapTypeBitmap8Bit;
MapKind mapKind = EXPLORERMAP;

rpos::core::RectangleF knownArea = platform_.getKnownArea(mapType, mapKind);
if (knownArea.width() <= 0 || knownArea.height() <= 0) {
RCLCPP_DEBUG(get_logger(), "No known map area yet");
return;
}

auto map = platform_.getMap(mapType, knownArea, mapKind);
auto dimension = map.getMapDimension();
auto resolution = map.getMapResolution();
const auto& data = map.getMapData();

int width = dimension.x();
int height = dimension.y();
if (width <= 0 || height <= 0 || data.empty()) return;

nav_msgs::msg::OccupancyGrid grid;
grid.header.stamp = this->now();
grid.header.frame_id = map_frame_id_;
grid.info.resolution = resolution.x();
grid.info.width = static_cast<uint32_t>(width);
grid.info.height = static_cast<uint32_t>(height);

grid.info.origin.position.x = knownArea.x();
grid.info.origin.position.y = knownArea.y();
grid.info.origin.position.z = 0.0;
grid.info.origin.orientation.w = 1.0;

// Slamware 8-bit EXPLORERMAP: 0=unknown, 1-127=free, 128-255=occupied
// OccupancyGrid: -1=unknown, 0=free, 100=occupied
grid.data.resize(static_cast<size_t>(width * height));

for (int row = 0; row < height; ++row) {
for (int col = 0; col < width; ++col) {
size_t src_idx = static_cast<size_t>(row * width + col);
size_t dst_idx = static_cast<size_t>(row * width + col);

if (src_idx >= data.size()) {
grid.data[dst_idx] = -1;
continue;
}

uint8_t pixel = data[src_idx];
if (pixel == 0) {
grid.data[dst_idx] = -1; // unknown
} else if (pixel < 128) {
grid.data[dst_idx] = 0; // free
} else {
grid.data[dst_idx] = 100; // occupied
}
}
}

map_pub_->publish(grid);

} catch (const std::exception& e) {
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
"Map update error: %s", e.what());
}
}

// Navigation is handled by Nav2. No internal goal/path logic here.

void publishStaticLaserTF()
{
double lx = get_parameter("laser_x_offset").as_double();
double ly = get_parameter("laser_y_offset").as_double();
double lz = get_parameter("laser_z_offset").as_double();

geometry_msgs::msg::TransformStamped tf;
tf.header.stamp = this->now();
tf.header.frame_id = base_frame_id_;
tf.child_frame_id = scan_frame_id_;
tf.transform.translation.x = lx;
tf.transform.translation.y = ly;
tf.transform.translation.z = lz;
tf.transform.rotation.w = 1.0;

static_tf_broadcaster_->sendTransform(tf);
RCLCPP_INFO(get_logger(), "Static TF: %s -> %s offset=(%.2f, %.2f, %.2f)",
base_frame_id_.c_str(), scan_frame_id_.c_str(), lx, ly, lz);
}

// ---- SDK ----
SlamwareCorePlatform platform_;
bool connected_ = false;

// ---- Parameters ----
std::string mapper_ip_;
int64_t mapper_port_;
std::string scan_frame_id_;
std::string map_frame_id_;
std::string base_frame_id_;
std::string odom_frame_id_;
double range_min_;
double range_max_;

// ---- Publishers ----
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

// ---- TF ----
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

// ---- Timers ----
rclcpp::TimerBase::SharedPtr scan_timer_;
rclcpp::TimerBase::SharedPtr map_timer_;
};

// ================================================================
int main(int argc, char* argv[])
{
rclcpp::init(argc, argv);
auto node = std::make_shared<SlamwareBridgeNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
