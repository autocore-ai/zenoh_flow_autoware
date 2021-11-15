#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>

AutowareAutoMsgsComplex32 Convert(const autoware_auto_msgs::msg::Complex32 &src) { return {src.real, src.imag}; }
AutowareAutoMsgsVehicleStateReport Convert(const autoware_auto_msgs::msg::VehicleStateReport &src) { return {Convert(src.stamp), src.fuel, src.blinker, src.headlight, src.wiper, src.gear, src.mode, src.hand_brake, src.horn}; }
AutowareAutoMsgsTrajectoryPoint Convert(const autoware_auto_msgs::msg::TrajectoryPoint &src) { return {Convert(src.time_from_start), src.x, src.y, src.z, Convert(src.heading), src.longitudinal_velocity_mps, src.lateral_velocity_mps, src.acceleration_mps2, src.heading_rate_rps, src.front_wheel_angle_rad, src.rear_wheel_angle_rad}; }
AutowareAutoMsgsVehicleKinematicState Convert(const autoware_auto_msgs::msg::VehicleKinematicState &src) { return {Convert(src.header), Convert(src.state), Convert(src.delta)}; }
BuiltinInterfacesDuration Convert(const builtin_interfaces::msg::Duration &src) { return {src.sec, src.nanosec}; }
BuiltinInterfacesTime Convert(const builtin_interfaces::msg::Time &src) { return {src.sec, src.nanosec}; }
GeometryMsgsQuaternion Convert(const geometry_msgs::msg::Quaternion &src) { return {src.x, src.y, src.z, src.w}; }
GeometryMsgsTransform Convert(const geometry_msgs::msg::Transform &src) { return {Convert(src.translation), Convert(src.rotation)}; }
GeometryMsgsVector3 Convert(const geometry_msgs::msg::Vector3 &src) { return {src.x, src.y, src.z}; }
StdMsgsHeader Convert(const std_msgs::msg::Header &src) { return {Convert(src.stamp), Convert(src.frame_id)}; }
builtin_interfaces::msg::Duration Convert(const BuiltinInterfacesDuration &src) { return builtin_interfaces::msg::Duration().set__sec(src.sec).set__nanosec(src.nanosec); }
builtin_interfaces::msg::Time Convert(const BuiltinInterfacesTime &src) { return builtin_interfaces::msg::Time().set__sec(src.sec).set__nanosec(src.nanosec); }
geometry_msgs::msg::Quaternion Convert(const GeometryMsgsQuaternion &src) { return geometry_msgs::msg::Quaternion().set__x(src.x).set__y(src.y).set__z(src.z).set__w(src.w); }
geometry_msgs::msg::Vector3 Convert(const GeometryMsgsVector3 &src) { return geometry_msgs::msg::Vector3().set__x(src.x).set__y(src.y).set__z(src.z); }
rust::cxxbridge1::String Convert(const std::string &src) { return rust::cxxbridge1::String(src.data(), src.size()); }
std::string Convert(const rust::cxxbridge1::String &src) { return std::string(src.data(), src.size()); }
std_msgs::msg::Header Convert(const StdMsgsHeader &src) { return std_msgs::msg::Header().set__stamp(Convert(src.stamp)).set__frame_id(Convert(src.frame_id)); }