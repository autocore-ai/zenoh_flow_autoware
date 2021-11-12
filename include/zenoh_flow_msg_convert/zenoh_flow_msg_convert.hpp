#pragma once
#include <autoware_auto.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
using namespace zenoh_flow::autoware_auto::ffi;

BuiltinInterfacesTime Convert(const builtin_interfaces::msg::Time &);
GeometryMsgsPoint Convert(const geometry_msgs::msg::Point &);
GeometryMsgsPose Convert(const geometry_msgs::msg::Pose &);
GeometryMsgsPoseStamped Convert(const geometry_msgs::msg::PoseStamped &);
GeometryMsgsPoseWithCovariance Convert(const geometry_msgs::msg::PoseWithCovariance &);
GeometryMsgsPoseWithCovarianceStamped Convert(const geometry_msgs::msg::PoseWithCovarianceStamped &);
GeometryMsgsQuaternion Convert(const geometry_msgs::msg::Quaternion &);
StdMsgsHeader Convert(const std_msgs::msg::Header &);
builtin_interfaces::msg::Time Convert(const BuiltinInterfacesTime &);
geometry_msgs::msg::Point Convert(const GeometryMsgsPoint &);
geometry_msgs::msg::Pose Convert(const GeometryMsgsPose &);
geometry_msgs::msg::PoseStamped Convert(const GeometryMsgsPoseStamped &);
geometry_msgs::msg::PoseWithCovariance Convert(const GeometryMsgsPoseWithCovariance &);
geometry_msgs::msg::PoseWithCovarianceStamped Convert(const GeometryMsgsPoseWithCovarianceStamped &);
geometry_msgs::msg::Quaternion Convert(const GeometryMsgsQuaternion &);
rust::cxxbridge1::String Convert(const std::string &);
std::string Convert(const rust::cxxbridge1::String &);
std_msgs::msg::Header Convert(const StdMsgsHeader &);
