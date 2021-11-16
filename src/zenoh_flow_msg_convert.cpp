#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>

AutowareAutoMsgsComplex32 Convert(const autoware_auto_msgs::msg::Complex32 &src) { return {src.real, src.imag}; }
AutowareAutoMsgsTrajectoryPoint Convert(const autoware_auto_msgs::msg::TrajectoryPoint &src) { return {Convert(src.time_from_start), src.x, src.y, src.z, Convert(src.heading), src.longitudinal_velocity_mps, src.lateral_velocity_mps, src.acceleration_mps2, src.heading_rate_rps, src.front_wheel_angle_rad, src.rear_wheel_angle_rad}; }
AutowareAutoMsgsVehicleControlCommand Convert(const autoware_auto_msgs::msg::VehicleControlCommand &src) { return {Convert(src.stamp), src.long_accel_mps2, src.velocity_mps, src.front_wheel_angle_rad, src.rear_wheel_angle_rad}; }
AutowareAutoMsgsVehicleKinematicState Convert(const autoware_auto_msgs::msg::VehicleKinematicState &src) { return {Convert(src.header), Convert(src.state), Convert(src.delta)}; }
AutowareAutoMsgsVehicleStateReport Convert(const autoware_auto_msgs::msg::VehicleStateReport &src) { return {Convert(src.stamp), src.fuel, src.blinker, src.headlight, src.wiper, src.gear, src.mode, src.hand_brake, src.horn}; }
BuiltinInterfacesDuration Convert(const builtin_interfaces::msg::Duration &src) { return {src.sec, src.nanosec}; }
BuiltinInterfacesTime Convert(const builtin_interfaces::msg::Time &src) { return {src.sec, src.nanosec}; }
GeometryMsgsPoint Convert(const geometry_msgs::msg::Point &src) { return {src.x, src.y, src.z}; }
GeometryMsgsPose Convert(const geometry_msgs::msg::Pose &src) { return {Convert(src.position), Convert(src.orientation)}; }
GeometryMsgsPoseStamped Convert(const geometry_msgs::msg::PoseStamped &src) { return {Convert(src.header), Convert(src.pose)}; }
GeometryMsgsPoseWithCovariance Convert(const geometry_msgs::msg::PoseWithCovariance &src) { return {Convert(src.pose), {}}; }
GeometryMsgsPoseWithCovarianceStamped Convert(const geometry_msgs::msg::PoseWithCovarianceStamped &src) { return {Convert(src.header), Convert(src.pose)}; }
GeometryMsgsQuaternion Convert(const geometry_msgs::msg::Quaternion &src) { return {src.x, src.y, src.z, src.w}; }
GeometryMsgsTransform Convert(const geometry_msgs::msg::Transform &src) { return {Convert(src.translation), Convert(src.rotation)}; }
GeometryMsgsVector3 Convert(const geometry_msgs::msg::Vector3 &src) { return {src.x, src.y, src.z}; }
StdMsgsHeader Convert(const std_msgs::msg::Header &src) { return {Convert(src.stamp), Convert(src.frame_id)}; }
autoware_auto_msgs::msg::Complex32 Convert(const AutowareAutoMsgsComplex32 &src) { return autoware_auto_msgs::msg::Complex32().set__real(src.real).set__imag(src.imag); }
autoware_auto_msgs::msg::TrajectoryPoint Convert(const AutowareAutoMsgsTrajectoryPoint &src) { return autoware_auto_msgs::msg::TrajectoryPoint().set__time_from_start(Convert(src.time_from_start)).set__x(src.x).set__y(src.y).set__z(src.z).set__heading(Convert(src.heading)).set__longitudinal_velocity_mps(src.longitudinal_velocity_mps).set__lateral_velocity_mps(src.lateral_velocity_mps).set__acceleration_mps2(src.acceleration_mps2).set__heading_rate_rps(src.heading_rate_rps).set__front_wheel_angle_rad(src.front_wheel_angle_rad).set__rear_wheel_angle_rad(src.rear_wheel_angle_rad); }
autoware_auto_msgs::msg::VehicleControlCommand Convert(const AutowareAutoMsgsVehicleControlCommand &src) { return autoware_auto_msgs::msg::VehicleControlCommand().set__stamp(Convert(src.stamp)).set__long_accel_mps2(src.long_accel_mps2).set__velocity_mps(src.velocity_mps).set__front_wheel_angle_rad(src.front_wheel_angle_rad).set__rear_wheel_angle_rad(src.rear_wheel_angle_rad); }
autoware_auto_msgs::msg::VehicleKinematicState Convert(const AutowareAutoMsgsVehicleKinematicState &src) { return autoware_auto_msgs::msg::VehicleKinematicState().set__header(Convert(src.header)).set__state(Convert(src.state)).set__delta(Convert(src.delta)); }
builtin_interfaces::msg::Duration Convert(const BuiltinInterfacesDuration &src) { return builtin_interfaces::msg::Duration().set__sec(src.sec).set__nanosec(src.nanosec); }
builtin_interfaces::msg::Time Convert(const BuiltinInterfacesTime &src) { return builtin_interfaces::msg::Time().set__sec(src.sec).set__nanosec(src.nanosec); }
geometry_msgs::msg::Point Convert(const GeometryMsgsPoint &src) { return geometry_msgs::msg::Point().set__x(src.x).set__y(src.y).set__z(src.z); }
geometry_msgs::msg::Pose Convert(const GeometryMsgsPose &src) { return geometry_msgs::msg::Pose().set__position(Convert(src.position)).set__orientation(Convert(src.orientation)); }
geometry_msgs::msg::PoseStamped Convert(const GeometryMsgsPoseStamped &src) { return geometry_msgs::msg::PoseStamped().set__header(Convert(src.header)).set__pose(Convert(src.pose)); }
geometry_msgs::msg::PoseWithCovariance Convert(const GeometryMsgsPoseWithCovariance &src) { return geometry_msgs::msg::PoseWithCovariance().set__pose(Convert(src.pose)).set__covariance({}); }
geometry_msgs::msg::PoseWithCovarianceStamped Convert(const GeometryMsgsPoseWithCovarianceStamped &src) { return geometry_msgs::msg::PoseWithCovarianceStamped().set__header(Convert(src.header)).set__pose(Convert(src.pose)); }
geometry_msgs::msg::Quaternion Convert(const GeometryMsgsQuaternion &src) { return geometry_msgs::msg::Quaternion().set__x(src.x).set__y(src.y).set__z(src.z).set__w(src.w); }
geometry_msgs::msg::Transform Convert(const GeometryMsgsTransform &src) { return geometry_msgs::msg::Transform().set__translation(Convert(src.translation)).set__rotation(Convert(src.rotation)); }
geometry_msgs::msg::Vector3 Convert(const GeometryMsgsVector3 &src) { return geometry_msgs::msg::Vector3().set__x(src.x).set__y(src.y).set__z(src.z); }
rust::cxxbridge1::String Convert(const std::string &src) { return rust::cxxbridge1::String(src.data(), src.size()); }
std::string Convert(const rust::cxxbridge1::String &src) { return std::string(src.data(), src.size()); }
std_msgs::msg::Header Convert(const StdMsgsHeader &src) { return std_msgs::msg::Header().set__stamp(Convert(src.stamp)).set__frame_id(Convert(src.frame_id)); }