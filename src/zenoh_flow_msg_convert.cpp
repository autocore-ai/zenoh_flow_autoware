// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>

AutowareAutoMsgsComplex32 Convert(const autoware_auto_msgs::msg::Complex32 &src) { return {src.real, src.imag}; }
AutowareAutoMsgsMapPrimitive Convert(const autoware_auto_msgs::msg::MapPrimitive &src) { return {src.id, Convert(src.primitive_type)}; }
AutowareAutoMsgsRoutePoint Convert(const autoware_auto_msgs::msg::RoutePoint &src) { return {Convert(src.position), Convert(src.heading)}; }
AutowareAutoMsgsTrajectoryPoint Convert(const autoware_auto_msgs::msg::TrajectoryPoint &src) { return {Convert(src.time_from_start), src.x, src.y, src.z, Convert(src.heading), src.longitudinal_velocity_mps, src.lateral_velocity_mps, src.acceleration_mps2, src.heading_rate_rps, src.front_wheel_angle_rad, src.rear_wheel_angle_rad}; }
AutowareAutoMsgsVehicleControlCommand Convert(const autoware_auto_msgs::msg::VehicleControlCommand &src) { return {Convert(src.stamp), src.long_accel_mps2, src.velocity_mps, src.front_wheel_angle_rad, src.rear_wheel_angle_rad}; }
AutowareAutoMsgsVehicleKinematicState Convert(const autoware_auto_msgs::msg::VehicleKinematicState &src) { return {Convert(src.header), Convert(src.state), Convert(src.delta)}; }
AutowareAutoMsgsVehicleStateCommand Convert(const autoware_auto_msgs::msg::VehicleStateCommand &src) { return {Convert(src.stamp), src.blinker, src.headlight, src.wiper, src.gear, src.mode, src.hand_brake, src.horn}; }
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
autoware_auto_msgs::msg::MapPrimitive Convert(const AutowareAutoMsgsMapPrimitive &src) { return autoware_auto_msgs::msg::MapPrimitive().set__id(src.id).set__primitive_type(Convert(src.primitive_type)); }
autoware_auto_msgs::msg::RoutePoint Convert(const AutowareAutoMsgsRoutePoint &src) { return autoware_auto_msgs::msg::RoutePoint().set__position(Convert(src.position)).set__heading(Convert(src.heading)); }
autoware_auto_msgs::msg::TrajectoryPoint Convert(const AutowareAutoMsgsTrajectoryPoint &src) { return autoware_auto_msgs::msg::TrajectoryPoint().set__time_from_start(Convert(src.time_from_start)).set__x(src.x).set__y(src.y).set__z(src.z).set__heading(Convert(src.heading)).set__longitudinal_velocity_mps(src.longitudinal_velocity_mps).set__lateral_velocity_mps(src.lateral_velocity_mps).set__acceleration_mps2(src.acceleration_mps2).set__heading_rate_rps(src.heading_rate_rps).set__front_wheel_angle_rad(src.front_wheel_angle_rad).set__rear_wheel_angle_rad(src.rear_wheel_angle_rad); }
autoware_auto_msgs::msg::VehicleControlCommand Convert(const AutowareAutoMsgsVehicleControlCommand &src) { return autoware_auto_msgs::msg::VehicleControlCommand().set__stamp(Convert(src.stamp)).set__long_accel_mps2(src.long_accel_mps2).set__velocity_mps(src.velocity_mps).set__front_wheel_angle_rad(src.front_wheel_angle_rad).set__rear_wheel_angle_rad(src.rear_wheel_angle_rad); }
autoware_auto_msgs::msg::VehicleKinematicState Convert(const AutowareAutoMsgsVehicleKinematicState &src) { return autoware_auto_msgs::msg::VehicleKinematicState().set__header(Convert(src.header)).set__state(Convert(src.state)).set__delta(Convert(src.delta)); }
autoware_auto_msgs::msg::VehicleStateCommand Convert(const AutowareAutoMsgsVehicleStateCommand &src) { return autoware_auto_msgs::msg::VehicleStateCommand().set__stamp(Convert(src.stamp)).set__blinker(src.blinker).set__headlight(src.headlight).set__wiper(src.wiper).set__gear(src.gear).set__mode(src.mode).set__hand_brake(src.hand_brake).set__horn(src.horn); }
autoware_auto_msgs::msg::VehicleStateReport Convert(const AutowareAutoMsgsVehicleStateReport &src) { return autoware_auto_msgs::msg::VehicleStateReport().set__stamp(Convert(src.stamp)).set__fuel(src.fuel).set__blinker(src.blinker).set__headlight(src.headlight).set__wiper(src.wiper).set__gear(src.gear).set__mode(src.mode).set__hand_brake(src.hand_brake).set__horn(src.horn); }
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
autoware_auto_msgs::msg::HADMapSegment Convert(const AutowareAutoMsgsHadmapSegment &src)
{
    auto primitives = std::vector<autoware_auto_msgs::msg::MapPrimitive>();
    for (auto &i : src.primitives)
    {
        primitives.push_back(Convert(i));
    }
    return autoware_auto_msgs::msg::HADMapSegment().set__primitives(primitives).set__preferred_primitive_id(src.preferred_primitive_id);
}
AutowareAutoMsgsHadmapSegment Convert(const autoware_auto_msgs::msg::HADMapSegment &src)
{
    auto primitives = ::rust::Vec<AutowareAutoMsgsMapPrimitive>();
    for (auto &i : src.primitives)
    {
        primitives.push_back(Convert(i));
    }
    return {primitives, src.preferred_primitive_id};
}
AutowareAutoMsgsTrajectory Convert(const autoware_auto_msgs::msg::Trajectory &src)
{
    auto points = ::rust::Vec<AutowareAutoMsgsTrajectoryPoint>();
    for (auto &i : src.points)
    {
        points.push_back(Convert(i));
    }
    return {Convert(src.header), points};
}
autoware_auto_msgs::msg::Trajectory Convert(const AutowareAutoMsgsTrajectory &src)
{
    auto points = rosidl_runtime_cpp::BoundedVector<autoware_auto_msgs::msg::TrajectoryPoint, 100>();
    for (auto &i : src.points)
    {
        points.push_back(Convert(i));
    }
    return autoware_auto_msgs::msg::Trajectory().set__header(Convert(src.header)).set__points(points);
}
autoware_auto_msgs::msg::HADMapRoute Convert(const AutowareAutoMsgsHadmapRoute &src)
{
    auto segments = std::vector<autoware_auto_msgs::msg::HADMapSegment>();
    for (auto &i : src.segments)
    {
        segments.push_back(Convert(i));
    }
    return autoware_auto_msgs::msg::HADMapRoute().set__header(Convert(src.header)).set__start_point(Convert(src.start_point)).set__goal_point(Convert(src.goal_point)).set__segments(segments);
}
AutowareAutoMsgsHadmapRoute Convert(const autoware_auto_msgs::msg::HADMapRoute &src)
{
    auto segments = ::rust::Vec<AutowareAutoMsgsHadmapSegment>();
    for (auto &i : src.segments)
    {
        segments.push_back(Convert(i));
    }
    return {Convert(src.header), Convert(src.start_point), Convert(src.goal_point), segments};
}
