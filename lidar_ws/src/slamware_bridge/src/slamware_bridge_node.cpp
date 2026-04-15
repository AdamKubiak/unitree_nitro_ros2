
/**
* slamware_bridge_node.cpp - ROS2 bridge for Slamtec Mapper M2M2
*
* Connects to the Mapper via Slamware SDK and publishes:
* /scan (sensor_msgs/LaserScan) - lidar data at ~10Hz
* /map (nav_msgs/OccupancyGrid) - occupancy map at ~0.5Hz
* /mapper/pose (geometry_msgs/PoseStamped) - robot pose at ~10Hz
* /mapper/nav_status (std_msgs/String) - navigation status
* /mapper/planned_path (nav_msgs/Path) - planned path for RViz
* /tf (map -> base_link transform)
*
* Subscribes to:
* /mapper/goal_pose (geometry_msgs/PoseStamped) - send navigation goal
* /mapper/cancel_goal (std_msgs/Empty) - cancel current navigation
*
* Publishes velocity commands to /cmd_vel to drive the robot along the
* Mapper's planned path (searchPath + pure pursuit follower).
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <functional>

using namespace rpos::robot_platforms;
using namespace rpos::features::location_provider;

// ================================================================
// Cached map data for local A* path planning
// ================================================================
struct MapCache {
std::vector<uint8_t> data;
int width = 0;
int height = 0;
float resolution = 0.05f;
float origin_x = 0.0f;
float origin_y = 0.0f;
bool valid = false;
std::mutex mtx;
};

class SlamwareBridgeNode : public rclcpp::Node
{
public:
SlamwareBridgeNode()
: Node("slamware_bridge")
{
// ---- Declare parameters ----
declare_parameter("mapper_ip", "192.168.11.1");
declare_parameter("mapper_port", 1445);
declare_parameter("scan_topic", "/scan");
declare_parameter("map_topic", "/map");
declare_parameter("cmd_vel_topic", "/cmd_vel");
declare_parameter("scan_rate", 10.0);
declare_parameter("map_rate", 0.5);
declare_parameter("scan_frame_id", "laser_frame");
declare_parameter("map_frame_id", "map");
declare_parameter("base_frame_id", "base_link");
declare_parameter("range_min", 0.1);
declare_parameter("range_max", 40.0);
// laser_frame offset from base_link (meters)
declare_parameter("laser_x_offset", 0.0);
declare_parameter("laser_y_offset", 0.0);
declare_parameter("laser_z_offset", 0.0);

// ---- Path follower parameters ----
declare_parameter("max_linear_vel", 0.3); // m/s - Go1 comfortable walk
declare_parameter("max_angular_vel", 0.8); // rad/s
declare_parameter("goal_tolerance", 0.15); // m - how close = reached
declare_parameter("waypoint_tolerance", 0.25); // m - advance to next waypoint
declare_parameter("heading_threshold", 0.5); // rad (~30deg) - rotate-in-place threshold
declare_parameter("path_follow_rate", 10.0); // Hz
declare_parameter("replan_interval", 5.0); // seconds between replans

// ---- Local planner parameters ----
declare_parameter("inflation_radius", 0.20); // m - obstacle inflation for safe clearance
declare_parameter("path_simplify_tolerance", 0.05); // m - RDP simplification tolerance

mapper_ip_ = get_parameter("mapper_ip").as_string();
mapper_port_ = get_parameter("mapper_port").as_int();
scan_frame_id_ = get_parameter("scan_frame_id").as_string();
map_frame_id_ = get_parameter("map_frame_id").as_string();
base_frame_id_ = get_parameter("base_frame_id").as_string();
range_min_ = get_parameter("range_min").as_double();
range_max_ = get_parameter("range_max").as_double();

max_linear_vel_ = get_parameter("max_linear_vel").as_double();
max_angular_vel_ = get_parameter("max_angular_vel").as_double();
goal_tolerance_ = get_parameter("goal_tolerance").as_double();
waypoint_tolerance_ = get_parameter("waypoint_tolerance").as_double();
heading_threshold_ = get_parameter("heading_threshold").as_double();
replan_interval_ = get_parameter("replan_interval").as_double();
inflation_radius_ = get_parameter("inflation_radius").as_double();
path_simplify_tol_ = get_parameter("path_simplify_tolerance").as_double();

// ---- Publishers ----
auto scan_topic = get_parameter("scan_topic").as_string();
auto map_topic = get_parameter("map_topic").as_string();
auto cmdvel_topic = get_parameter("cmd_vel_topic").as_string();

auto scan_qos = rclcpp::QoS(10).reliable();
scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, scan_qos);
pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("mapper/pose", scan_qos);

// Map: transient_local so late-joining subscribers get the latest map
auto map_qos = rclcpp::QoS(1).transient_local().reliable();
map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);

status_pub_ = create_publisher<std_msgs::msg::String>("mapper/nav_status", 10);
cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmdvel_topic, 10);
path_pub_ = create_publisher<nav_msgs::msg::Path>("mapper/planned_path", 10);

// ---- TF broadcasters ----
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

// Publish static TF: base_link -> laser_frame
publishStaticLaserTF();

// ---- Subscribers ----
goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
"mapper/goal_pose", 10,
std::bind(&SlamwareBridgeNode::goalCallback, this, std::placeholders::_1));

cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
"mapper/cancel_goal", 10,
std::bind(&SlamwareBridgeNode::cancelCallback, this, std::placeholders::_1));

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

// ---- Start timers ----
double scan_rate = get_parameter("scan_rate").as_double();
double map_rate = get_parameter("map_rate").as_double();
double path_follow_rate = get_parameter("path_follow_rate").as_double();

scan_timer_ = create_wall_timer(
std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate)),
std::bind(&SlamwareBridgeNode::scanTimerCallback, this));

map_timer_ = create_wall_timer(
std::chrono::milliseconds(static_cast<int>(1000.0 / map_rate)),
std::bind(&SlamwareBridgeNode::mapTimerCallback, this));

nav_timer_ = create_wall_timer(
std::chrono::milliseconds(static_cast<int>(1000.0 / path_follow_rate)),
std::bind(&SlamwareBridgeNode::navTimerCallback, this));

RCLCPP_INFO(get_logger(),
"Slamware Bridge ready - scan %.0fHz, map %.1fHz, nav %.0fHz",
scan_rate, map_rate, path_follow_rate);
RCLCPP_INFO(get_logger(),
"Path follower: max_v=%.2f m/s, max_w=%.2f rad/s, goal_tol=%.2f m",
max_linear_vel_, max_angular_vel_, goal_tolerance_);
RCLCPP_INFO(get_logger(),
"Planner: LOCAL A*, inflation=%.2fm, simplify_tol=%.3fm",
inflation_radius_, path_simplify_tol_);
}

~SlamwareBridgeNode()
{
stopRobot();
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

geometry_msgs::msg::TransformStamped tf_msg;
tf_msg.header.stamp = stamp;
tf_msg.header.frame_id = map_frame_id_;
tf_msg.child_frame_id = base_frame_id_;
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

// Cache raw map data for the local A* path planner
{
std::lock_guard<std::mutex> lock(map_cache_.mtx);
map_cache_.data = data;
map_cache_.width = width;
map_cache_.height = height;
map_cache_.resolution = resolution.x();
map_cache_.origin_x = knownArea.x();
map_cache_.origin_y = knownArea.y();
map_cache_.valid = true;
}

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

// ================================================================
// Goal handling - plan path using searchPath()
// ================================================================
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
if (!connected_) {
RCLCPP_WARN(get_logger(), "Not connected to Mapper, ignoring goal");
return;
}

float goal_x = static_cast<float>(msg->pose.position.x);
float goal_y = static_cast<float>(msg->pose.position.y);

RCLCPP_INFO(get_logger(), "Navigation goal received: (%.2f, %.2f)", goal_x, goal_y);

// Stop any current navigation
stopRobot();

goal_location_ = rpos::core::Location(goal_x, goal_y);

// Try to get a planned path from the Mapper
if (!planPath()) {
// searchPath failed - fall back to direct single-waypoint path
RCLCPP_WARN(get_logger(),
"searchPath not available, using direct waypoint to goal");
planned_waypoints_.clear();
planned_waypoints_.push_back(goal_location_);
}

current_waypoint_idx_ = 0;
navigating_ = true;
last_replan_time_ = this->now();
log_counter_ = 0;

publishPlannedPath();
publishStatus("NAVIGATING");

RCLCPP_INFO(get_logger(), "Following path with %zu waypoints to (%.2f, %.2f)",
planned_waypoints_.size(), goal_x, goal_y);
}

void cancelCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
RCLCPP_INFO(get_logger(), "Navigation cancelled by request");
stopRobot();
navigating_ = false;
planned_waypoints_.clear();
publishStatus("CANCELLED");
}

// ================================================================
// Path planning - local A* on cached occupancy grid
// ================================================================
bool planPath()
{
return planPathLocal();
}

/** Local A* path planning on the cached occupancy grid */
bool planPathLocal()
{
rpos::core::Pose pose;
try {
pose = platform_.getPose();
} catch (const std::exception& e) {
RCLCPP_WARN(get_logger(), "Cannot get pose for local planner: %s", e.what());
return false;
}

float start_x = pose.x();
float start_y = pose.y();
float goal_x = goal_location_.x();
float goal_y = goal_location_.y();

// Lock and copy the map cache
std::vector<uint8_t> map_data;
int map_w, map_h;
float map_res, map_ox, map_oy;
{
std::lock_guard<std::mutex> lock(map_cache_.mtx);
if (!map_cache_.valid) {
RCLCPP_WARN(get_logger(), "Local planner: no map data yet, using direct waypoint");
planned_waypoints_.clear();
planned_waypoints_.push_back(goal_location_);
return true;
}
map_data = map_cache_.data;
map_w = map_cache_.width;
map_h = map_cache_.height;
map_res = map_cache_.resolution;
map_ox = map_cache_.origin_x;
map_oy = map_cache_.origin_y;
}

// Build inflated costmap (true = obstacle/inflated)
std::vector<bool> costmap(map_w * map_h, false);
int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / map_res));

for (int r = 0; r < map_h; ++r) {
for (int c = 0; c < map_w; ++c) {
size_t idx = static_cast<size_t>(r * map_w + c);
if (idx >= map_data.size()) continue;
uint8_t pixel = map_data[idx];
// Slamware 8-bit EXPLORERMAP: 0=unknown, 1-127=free, 128-255=occupied
if (pixel >= 128) {
// Inflate obstacle
for (int dr = -inflate_cells; dr <= inflate_cells; ++dr) {
for (int dc = -inflate_cells; dc <= inflate_cells; ++dc) {
if (dr * dr + dc * dc > inflate_cells * inflate_cells) continue;
int nr = r + dr, nc = c + dc;
if (nr >= 0 && nr < map_h && nc >= 0 && nc < map_w) {
costmap[nr * map_w + nc] = true;
}
}
}
}
}
}

// Convert world -> grid
auto worldToGrid = [&](float wx, float wy, int& gx, int& gy) {
gx = static_cast<int>((wx - map_ox) / map_res);
gy = static_cast<int>((wy - map_oy) / map_res);
};
auto gridToWorld = [&](int gx, int gy, float& wx, float& wy) {
wx = map_ox + (static_cast<float>(gx) + 0.5f) * map_res;
wy = map_oy + (static_cast<float>(gy) + 0.5f) * map_res;
};

int sx, sy, gx, gy;
worldToGrid(start_x, start_y, sx, sy);
worldToGrid(goal_x, goal_y, gx, gy);

// Clamp to map bounds
sx = std::clamp(sx, 0, map_w - 1);
sy = std::clamp(sy, 0, map_h - 1);
gx = std::clamp(gx, 0, map_w - 1);
gy = std::clamp(gy, 0, map_h - 1);

// Ensure start cell is traversable — the robot is physically here,
// so clear inflation (but NOT real obstacles) around its position.
// Without this, A* cannot begin expanding when near walls.
if (costmap[sy * map_w + sx]) {
RCLCPP_WARN(get_logger(),
"Start cell (%d,%d) in inflated zone, clearing inflation around robot", sx, sy);
int clear_r = inflate_cells + 2; // slightly larger than inflation
for (int dr = -clear_r; dr <= clear_r; ++dr) {
for (int dc = -clear_r; dc <= clear_r; ++dc) {
int nr = sy + dr, nc = sx + dc;
if (nr >= 0 && nr < map_h && nc >= 0 && nc < map_w) {
size_t idx = static_cast<size_t>(nr * map_w + nc);
if (idx < map_data.size()) {
uint8_t pixel = map_data[idx];
// Only clear if not a real obstacle (keep pixel >= 128 blocked)
if (pixel < 128) {
costmap[nr * map_w + nc] = false;
}
}
}
}
}
}

// If goal cell is in obstacle, find nearest free cell
if (costmap[gy * map_w + gx]) {
RCLCPP_WARN(get_logger(), "Goal is in obstacle/inflated zone, searching nearby free cell");
bool found = false;
for (int radius = 1; radius < std::max(map_w, map_h) && !found; ++radius) {
for (int dr = -radius; dr <= radius && !found; ++dr) {
for (int dc = -radius; dc <= radius && !found; ++dc) {
if (std::abs(dr) != radius && std::abs(dc) != radius) continue;
int nr = gy + dr, nc = gx + dc;
if (nr >= 0 && nr < map_h && nc >= 0 && nc < map_w &&
!costmap[nr * map_w + nc]) {
gx = nc;
gy = nr;
found = true;
}
}
}
}
if (!found) {
RCLCPP_ERROR(get_logger(), "Local planner: no free cell near goal");
return false;
}
}

// ---- A* search (8-connected) ----
struct Cell {
int x, y;
float g, f;
bool operator>(const Cell& o) const { return f > o.f; }
};

auto heuristic = [](int x1, int y1, int x2, int y2) -> float {
float dx = static_cast<float>(x1 - x2);
float dy = static_cast<float>(y1 - y2);
return std::sqrt(dx * dx + dy * dy);
};

auto cellKey = [&](int x, int y) -> int { return y * map_w + x; };

std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open;
std::unordered_map<int, float> g_score;
std::unordered_map<int, int> came_from; // key -> parent key

int start_key = cellKey(sx, sy);
int goal_key = cellKey(gx, gy);

open.push({sx, sy, 0.0f, heuristic(sx, sy, gx, gy)});
g_score[start_key] = 0.0f;

// 8-connected neighbors
const int dx8[] = {1, -1, 0, 0, 1, -1, 1, -1};
const int dy8[] = {0, 0, 1, -1, 1, 1, -1, -1};
const float cost8[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

bool path_found = false;
int iterations = 0;
const int max_iterations = map_w * map_h; // safety limit

while (!open.empty() && iterations++ < max_iterations) {
Cell cur = open.top();
open.pop();

int cur_key = cellKey(cur.x, cur.y);
if (cur_key == goal_key) {
path_found = true;
break;
}

// Skip if we already found a better path to this cell
auto it = g_score.find(cur_key);
if (it != g_score.end() && cur.g > it->second) continue;

for (int d = 0; d < 8; ++d) {
int nx = cur.x + dx8[d];
int ny = cur.y + dy8[d];
if (nx < 0 || nx >= map_w || ny < 0 || ny >= map_h) continue;
if (costmap[ny * map_w + nx]) continue; // obstacle

// For diagonal moves, also check the two adjacent cardinal cells
if (d >= 4) {
if (costmap[cur.y * map_w + nx] || costmap[ny * map_w + cur.x])
continue;
}

float new_g = cur.g + cost8[d];
int n_key = cellKey(nx, ny);
auto git = g_score.find(n_key);
if (git == g_score.end() || new_g < git->second) {
g_score[n_key] = new_g;
came_from[n_key] = cur_key;
open.push({nx, ny, new_g, new_g + heuristic(nx, ny, gx, gy)});
}
}
}

if (!path_found) {
RCLCPP_WARN(get_logger(), "Local A*: no path found after %d iterations", iterations);
// Fallback: direct waypoint
planned_waypoints_.clear();
planned_waypoints_.push_back(goal_location_);
return true;
}

// Reconstruct path (grid coords)
std::vector<std::pair<int,int>> grid_path;
int key = goal_key;
while (key != start_key) {
int cy = key / map_w;
int cx = key % map_w;
grid_path.push_back({cx, cy});
auto cit = came_from.find(key);
if (cit == came_from.end()) break;
key = cit->second;
}
std::reverse(grid_path.begin(), grid_path.end());

// Convert to world coordinates
std::vector<rpos::core::Location> raw_path;
raw_path.reserve(grid_path.size());
for (const auto& [cx, cy] : grid_path) {
float wx, wy;
gridToWorld(cx, cy, wx, wy);
raw_path.emplace_back(wx, wy);
}

// Simplify path using Ramer-Douglas-Peucker
planned_waypoints_ = simplifyPath(raw_path, path_simplify_tol_);

// Always ensure the exact goal is the last waypoint
if (!planned_waypoints_.empty()) {
planned_waypoints_.back() = goal_location_;
} else {
planned_waypoints_.push_back(goal_location_);
}

RCLCPP_INFO(get_logger(), "Local A*: %zu raw -> %zu simplified waypoints (iterations=%d)",
raw_path.size(), planned_waypoints_.size(), iterations);
return true;
}

// ================================================================
// Navigation timer - Pure Pursuit path follower (~10Hz)
// ================================================================
void navTimerCallback()
{
if (!connected_ || !navigating_ || planned_waypoints_.empty()) return;

// --- Get current robot pose from the Mapper ---
rpos::core::Pose pose;
try {
pose = platform_.getPose();
} catch (const std::exception& e) {
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
"Pose unavailable during nav: %s", e.what());
return;
}

float robot_x = pose.x();
float robot_y = pose.y();
float robot_yaw = pose.yaw();

// --- Check if we reached the final goal ---
float dx_goal = goal_location_.x() - robot_x;
float dy_goal = goal_location_.y() - robot_y;
float dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

if (dist_to_goal < goal_tolerance_) {
stopRobot();
navigating_ = false;
RCLCPP_INFO(get_logger(), "Goal reached! (dist=%.3f m)", dist_to_goal);
publishStatus("REACHED");
return;
}

// --- Final approach mode: when close to goal, steer directly to it ---
double final_approach_dist = goal_tolerance_ * 3.0; // ~0.45m
if (dist_to_goal < final_approach_dist) {
// Point directly at the goal, ignore waypoints
float target_yaw = std::atan2(dy_goal, dx_goal);
float yaw_error = normalizeAngle(target_yaw - robot_yaw);

if (std::abs(yaw_error) > M_PI_2) {
// Rotate in place — NO forward creep when near goal
stopRobot();
navigating_ = false;
return;
}

geometry_msgs::msg::Twist cmd;

double scale = static_cast<double>(dist_to_goal) / final_approach_dist;
cmd.linear.x = clampVal(max_linear_vel_*0.3*scale*scale, 0.0, 0.08);
cmd.angular.z = clampVal(yaw_error*1.0, -max_angular_vel_*0.3, max_angular_vel_*0.3);

cmd_vel_pub_->publish(cmd);

if (++log_counter_ % 50 == 0) {
RCLCPP_INFO(get_logger(),
"Final approach: dist=%.3f yaw_err=%.1fdeg v=%.2f w=%.2f",
dist_to_goal, yaw_error * 180.0 / M_PI,
cmd.linear.x, cmd.angular.z);
}
return; // skip replanning during final approach
}

// --- Advance past waypoints we've already reached ---
while (current_waypoint_idx_ < planned_waypoints_.size() - 1) {
float dx = planned_waypoints_[current_waypoint_idx_].x() - robot_x;
float dy = planned_waypoints_[current_waypoint_idx_].y() - robot_y;
float dist = std::sqrt(dx * dx + dy * dy);
if (dist < waypoint_tolerance_) {
current_waypoint_idx_++;
RCLCPP_DEBUG(get_logger(), "Waypoint %zu reached, advancing",
current_waypoint_idx_ - 1);
} else {
break;
}
}

// --- Compute heading and distance to current target waypoint ---
const auto& target = planned_waypoints_[current_waypoint_idx_];
float dx = target.x() - robot_x;
float dy = target.y() - robot_y;
float dist_to_wp = std::sqrt(dx * dx + dy * dy);

float target_yaw = std::atan2(dy, dx);
float yaw_error = normalizeAngle(target_yaw - robot_yaw);

// --- Compute velocity command ---
geometry_msgs::msg::Twist cmd;

if (std::abs(yaw_error) > heading_threshold_) {
// Large heading error: rotate in place
cmd.linear.x = 0.0;
cmd.angular.z = clampVal(yaw_error * 1.5, -max_angular_vel_, max_angular_vel_);
} else {
// Drive forward, steering proportionally to heading error
double speed_scale = std::cos(yaw_error);
// Slow down when approaching the goal
double approach_scale = std::min(1.0, static_cast<double>(dist_to_goal) / final_approach_dist);
cmd.linear.x = clampVal(speed_scale * approach_scale * max_linear_vel_,
0.0, max_linear_vel_);
cmd.angular.z = clampVal(yaw_error * 2.0, -max_angular_vel_, max_angular_vel_);
}

cmd_vel_pub_->publish(cmd);

// --- Periodic status logging (every ~5s at 10Hz) ---
if (++log_counter_ % 50 == 0) {
RCLCPP_INFO(get_logger(),
"Nav: wp=%zu/%zu dist_goal=%.2f dist_wp=%.2f yaw_err=%.1fdeg v=%.2f w=%.2f",
current_waypoint_idx_, planned_waypoints_.size(),
dist_to_goal, dist_to_wp,
yaw_error * 180.0 / M_PI,
cmd.linear.x, cmd.angular.z);
}

// --- Periodic replanning (only when far from goal) ---
auto now_time = this->now();
if ((now_time - last_replan_time_).seconds() > replan_interval_) {
RCLCPP_DEBUG(get_logger(), "Replanning path...");
if (planPath()) {
current_waypoint_idx_ = findClosestWaypointAhead(robot_x, robot_y, robot_yaw);
publishPlannedPath();
RCLCPP_DEBUG(get_logger(), "Replan: %zu waypoints, starting at %zu",
planned_waypoints_.size(), current_waypoint_idx_);
}
last_replan_time_ = now_time;
}
}

// ================================================================
// Helpers
// ================================================================
void stopRobot()
{
geometry_msgs::msg::Twist stop;
stop.linear.x = 0.0;
stop.linear.y = 0.0;
stop.linear.z = 0.0;
stop.angular.x = 0.0;
stop.angular.y = 0.0;
stop.angular.z = 0.0;
cmd_vel_pub_->publish(stop);
}

void publishStatus(const std::string& status)
{
std_msgs::msg::String msg;
msg.data = status;
status_pub_->publish(msg);
}

void publishPlannedPath()
{
nav_msgs::msg::Path path_msg;
path_msg.header.stamp = this->now();
path_msg.header.frame_id = map_frame_id_;

for (const auto& wp : planned_waypoints_) {
geometry_msgs::msg::PoseStamped ps;
ps.header = path_msg.header;
ps.pose.position.x = wp.x();
ps.pose.position.y = wp.y();
ps.pose.position.z = 0.0;
ps.pose.orientation.w = 1.0;
path_msg.poses.push_back(ps);
}

path_pub_->publish(path_msg);
}

/** Find the closest waypoint that is ahead of the robot (not already passed). */
size_t findClosestWaypointAhead(float rx, float ry, float ryaw)
{
size_t best = 0;
float best_dist = std::numeric_limits<float>::max();

for (size_t i = 0; i < planned_waypoints_.size(); ++i) {
float dx = planned_waypoints_[i].x() - rx;
float dy = planned_waypoints_[i].y() - ry;
float dist = std::sqrt(dx * dx + dy * dy);

// Skip waypoints we've already passed (too close, not the last)
if (dist < waypoint_tolerance_ && i < planned_waypoints_.size() - 1) {
continue;
}
if (dist < best_dist) {
best_dist = dist;
best = i;
}
}
return best;
}

static float normalizeAngle(float angle)
{
while (angle > M_PI) angle -= 2.0f * static_cast<float>(M_PI);
while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
return angle;
}

static double clampVal(double val, double lo, double hi)
{
if (val < lo) return lo;
if (val > hi) return hi;
return val;
}

/** Ramer-Douglas-Peucker path simplification */
static std::vector<rpos::core::Location> simplifyPath(
const std::vector<rpos::core::Location>& path, float epsilon)
{
if (path.size() <= 2) return path;

// Find the point with the maximum distance from the line (first, last)
float dmax = 0.0f;
size_t index = 0;
size_t end = path.size() - 1;

float lx = path[end].x() - path[0].x();
float ly = path[end].y() - path[0].y();
float line_len = std::sqrt(lx * lx + ly * ly);

for (size_t i = 1; i < end; ++i) {
float d;
if (line_len < 1e-6f) {
float dx = path[i].x() - path[0].x();
float dy = path[i].y() - path[0].y();
d = std::sqrt(dx * dx + dy * dy);
} else {
d = std::abs(lx * (path[0].y() - path[i].y()) -
ly * (path[0].x() - path[i].x())) / line_len;
}
if (d > dmax) {
index = i;
dmax = d;
}
}

if (dmax > epsilon) {
std::vector<rpos::core::Location> left(path.begin(), path.begin() + index + 1);
std::vector<rpos::core::Location> right(path.begin() + index, path.end());
auto r1 = simplifyPath(left, epsilon);
auto r2 = simplifyPath(right, epsilon);
r1.insert(r1.end(), r2.begin() + 1, r2.end());
return r1;
} else {
return {path.front(), path.back()};
}
}

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

// ---- Map cache for local planner ----
MapCache map_cache_;

// ---- Path following state ----
std::vector<rpos::core::Location> planned_waypoints_;
size_t current_waypoint_idx_ = 0;
rpos::core::Location goal_location_;
bool navigating_ = false;
rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
int log_counter_ = 0;

// ---- Parameters ----
std::string mapper_ip_;
int64_t mapper_port_;
std::string scan_frame_id_;
std::string map_frame_id_;
std::string base_frame_id_;
double range_min_;
double range_max_;
double max_linear_vel_;
double max_angular_vel_;
double goal_tolerance_;
double waypoint_tolerance_;
double heading_threshold_;
double replan_interval_;
double inflation_radius_;
double path_simplify_tol_;

// ---- Publishers ----
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

// ---- TF ----
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

// ---- Subscribers ----
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;

// ---- Timers ----
rclcpp::TimerBase::SharedPtr scan_timer_;
rclcpp::TimerBase::SharedPtr map_timer_;
rclcpp::TimerBase::SharedPtr nav_timer_;
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
