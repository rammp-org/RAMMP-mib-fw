#pragma once

/// @file ros_messages.hpp
/// @brief Standard ROS 2 messages the MIB publishes directly to ROS 2 peers.
///
/// Each struct mirrors its ROS 2 .msg definition field for field and in order:
/// espp serializes it as XCDR1, which is what ROS 2 (rmw_fastrtps,
/// rmw_cyclonedds) puts on the wire. Do not reorder, add, or retype fields; a
/// ROS 2 subscriber would mis-decode the sample.

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "messages/topic.hpp"

namespace mib::ros {

namespace builtin_interfaces {
/// builtin_interfaces/msg/Time
struct Time {
  int32_t sec{0};
  uint32_t nanosec{0};
};
} // namespace builtin_interfaces

namespace std_msgs {
/// std_msgs/msg/Header
struct Header {
  builtin_interfaces::Time stamp{};
  std::string frame_id{};
};
} // namespace std_msgs

namespace geometry_msgs {
/// geometry_msgs/msg/Point
struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// geometry_msgs/msg/Vector3
struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// geometry_msgs/msg/Quaternion
struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// geometry_msgs/msg/Pose
struct Pose {
  Point position{};
  Quaternion orientation{};
};

/// geometry_msgs/msg/PoseWithCovariance
struct PoseWithCovariance {
  Pose pose{};
  std::array<double, 36> covariance{}; ///< Row-major 6x6: x, y, z, roll, pitch, yaw.
};

/// geometry_msgs/msg/Twist
struct Twist {
  Vector3 linear{};
  Vector3 angular{};
};

/// geometry_msgs/msg/TwistWithCovariance
struct TwistWithCovariance {
  Twist twist{};
  std::array<double, 36> covariance{}; ///< Row-major 6x6: vx, vy, vz, wx, wy, wz.
};

/// geometry_msgs/msg/Transform
struct Transform {
  Vector3 translation{};
  Quaternion rotation{};
};

/// geometry_msgs/msg/TransformStamped
struct TransformStamped {
  std_msgs::Header header{};
  std::string child_frame_id{};
  Transform transform{};
};
} // namespace geometry_msgs

namespace nav_msgs {
/// nav_msgs/msg/Odometry
struct Odometry {
  std_msgs::Header header{};
  std::string child_frame_id{};
  geometry_msgs::PoseWithCovariance pose{};   ///< In header.frame_id.
  geometry_msgs::TwistWithCovariance twist{}; ///< In child_frame_id.
};
} // namespace nav_msgs

namespace tf2_msgs {
/// tf2_msgs/msg/TFMessage
struct TFMessage {
  std::vector<geometry_msgs::TransformStamped> transforms{};
};
} // namespace tf2_msgs

/// ROS 2 /odom, typed. ROS 2 prefixes DDS topic names with "rt".
inline constexpr rammp::Topic<nav_msgs::Odometry> kOdom{"rt/odom",
                                                        "nav_msgs::msg::dds_::Odometry_"};

/// ROS 2 /tf, typed.
inline constexpr rammp::Topic<tf2_msgs::TFMessage> kTf{"rt/tf", "tf2_msgs::msg::dds_::TFMessage_"};

} // namespace mib::ros
