#pragma once
#include <msgs.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>
using namespace zenoh_flow::autoware_auto::ffi;

AutowareAutoMsgsHadmapRoute Convert(const autoware_auto_msgs::msg::HADMapRoute &);
AutowareAutoMsgsTrajectory Convert(const autoware_auto_msgs::msg::Trajectory &);
AutowareAutoMsgsVehicleControlCommand Convert(const autoware_auto_msgs::msg::VehicleControlCommand &);
AutowareAutoMsgsVehicleKinematicState Convert(const autoware_auto_msgs::msg::VehicleKinematicState &);
AutowareAutoMsgsVehicleStateCommand Convert(const autoware_auto_msgs::msg::VehicleStateCommand &);
AutowareAutoMsgsVehicleStateReport Convert(const autoware_auto_msgs::msg::VehicleStateReport &);
BuiltinInterfacesDuration Convert(const builtin_interfaces::msg::Duration &);
BuiltinInterfacesTime Convert(const builtin_interfaces::msg::Time &);
GeometryMsgsPoint Convert(const geometry_msgs::msg::Point &);
GeometryMsgsPose Convert(const geometry_msgs::msg::Pose &);
GeometryMsgsPoseStamped Convert(const geometry_msgs::msg::PoseStamped &);
GeometryMsgsPoseWithCovariance Convert(const geometry_msgs::msg::PoseWithCovariance &);
GeometryMsgsPoseWithCovarianceStamped Convert(const geometry_msgs::msg::PoseWithCovarianceStamped &);
GeometryMsgsQuaternion Convert(const geometry_msgs::msg::Quaternion &);
GeometryMsgsTransform Convert(const geometry_msgs::msg::Transform &);
GeometryMsgsVector3 Convert(const geometry_msgs::msg::Vector3 &);
StdMsgsHeader Convert(const std_msgs::msg::Header &);
autoware_auto_msgs::msg::HADMapRoute Convert(const AutowareAutoMsgsHadmapRoute &);
autoware_auto_msgs::msg::Trajectory Convert(const AutowareAutoMsgsTrajectory &);
autoware_auto_msgs::msg::VehicleControlCommand Convert(const AutowareAutoMsgsVehicleControlCommand &);
autoware_auto_msgs::msg::VehicleKinematicState Convert(const AutowareAutoMsgsVehicleKinematicState &);
autoware_auto_msgs::msg::VehicleStateCommand Convert(const AutowareAutoMsgsVehicleStateCommand &);
autoware_auto_msgs::msg::VehicleStateReport Convert(const AutowareAutoMsgsVehicleStateReport &);
builtin_interfaces::msg::Duration Convert(const BuiltinInterfacesDuration &);
builtin_interfaces::msg::Time Convert(const BuiltinInterfacesTime &);
geometry_msgs::msg::Point Convert(const GeometryMsgsPoint &);
geometry_msgs::msg::Pose Convert(const GeometryMsgsPose &);
geometry_msgs::msg::PoseStamped Convert(const GeometryMsgsPoseStamped &);
geometry_msgs::msg::PoseWithCovariance Convert(const GeometryMsgsPoseWithCovariance &);
geometry_msgs::msg::PoseWithCovarianceStamped Convert(const GeometryMsgsPoseWithCovarianceStamped &);
geometry_msgs::msg::Quaternion Convert(const GeometryMsgsQuaternion &);
geometry_msgs::msg::Transform Convert(const GeometryMsgsTransform &);
geometry_msgs::msg::Vector3 Convert(const GeometryMsgsVector3 &);
rust::cxxbridge1::String Convert(const std::string &);
std::string Convert(const rust::cxxbridge1::String &);
std_msgs::msg::Header Convert(const StdMsgsHeader &);
