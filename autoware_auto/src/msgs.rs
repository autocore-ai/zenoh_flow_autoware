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

/// ROS msgs used in Autoware.Auto avp demo
///
/// Native ROS msg can be found on [GitHub](https://github.com/ros2)
///
/// Autoware.Auto msg can be found on [GitLab](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/tree/34d98a6173ae02572fef10ebc47a2916c2b0d395)
#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    /// Complex32.msg
    ///
    /// Can be used to represent yaw angle for trajectories
    ///
    /// To convert back to a yaw angle, see
    ///
    /// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsComplex32 {
        /// cos(yaw / 2)
        pub real: f32,
        /// sin(yaw / 2)
        pub imag: f32,
    }
    /// A route within a high-definition map defined by
    /// the start and goal points and map primitives
    /// describing the route between the two.
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsHadmapRoute {
        pub header: StdMsgsHeader,
        /// The start_point must exist within the bounds of the primitives in the first
        /// segment defined in the route_segments array.
        pub start_point: AutowareAutoMsgsRoutePoint,
        /// The goal_point must exist within the bounds of the primitives in the last
        /// segment defined in the route_semgents array.
        pub goal_point: AutowareAutoMsgsRoutePoint,
        pub segments: Vec<AutowareAutoMsgsHadmapSegment>,
    }
    /// A segment of an HADMap which contains one or more MapPrimitives.
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsHadmapSegment {
        pub primitives: Vec<AutowareAutoMsgsMapPrimitive>,
        pub preferred_primitive_id: i64,
    }
    /// Map primitive information
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsMapPrimitive {
        pub id: i64,
        /// Type of primitive, such as lane, polygon, line.
        pub primitive_type: String,
    }
    /// Representation of a position and heading
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsRoutePoint {
        /// This position is the position of the CoG frame origin of
        /// the vehicle with relation to the frame in which the Route exists.
        pub position: GeometryMsgsPoint,
        /// This heading is relative to the X or East-facing axis of
        /// the frame in which the Route exists.
        pub heading: AutowareAutoMsgsComplex32,
    }
    /// A set of trajectory points for the controller
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsTrajectory {
        pub header: StdMsgsHeader,
        pub points: Vec<AutowareAutoMsgsTrajectoryPoint>,
    }
    /// Representation of a trajectory point for the controller
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsTrajectoryPoint {
        pub time_from_start: BuiltinInterfacesDuration,
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub heading: AutowareAutoMsgsComplex32,
        pub longitudinal_velocity_mps: f32,
        pub lateral_velocity_mps: f32,
        pub acceleration_mps2: f32,
        pub heading_rate_rps: f32,
        pub front_wheel_angle_rad: f32,
        pub rear_wheel_angle_rad: f32,
    }
    /// Information that is sent to Vehicle interface
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleControlCommand {
        pub stamp: BuiltinInterfacesTime,
        /// should be negative when reversed
        pub long_accel_mps2: f32,
        /// should be negative when reversed
        pub velocity_mps: f32,
        pub front_wheel_angle_rad: f32,
        pub rear_wheel_angle_rad: f32,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleKinematicState {
        pub header: StdMsgsHeader,
        pub state: AutowareAutoMsgsTrajectoryPoint,
        pub delta: GeometryMsgsTransform,
    }
    // TODO: use enum types : https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/34d98a6173ae02572fef10ebc47a2916c2b0d395/autoware_auto_msgs/msg/VehicleStateCommand.idl
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleStateCommand {
        pub stamp: BuiltinInterfacesTime,
        pub blinker: u8,
        pub headlight: u8,
        pub wiper: u8,
        pub gear: u8,
        pub mode: u8,
        pub hand_brake: bool,
        pub horn: bool,
    }
    // TODO: use enum types : https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/34d98a6173ae02572fef10ebc47a2916c2b0d395/autoware_auto_msgs/msg/VehicleStateReport.idl
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleStateReport {
        pub stamp: BuiltinInterfacesTime,
        /// 0 to 100
        pub fuel: u8,
        pub blinker: u8,
        pub headlight: u8,
        pub wiper: u8,
        pub gear: u8,
        pub mode: u8,
        pub hand_brake: bool,
        pub horn: bool,
    }
    /// Duration defines a period between two time points. It is comprised of a
    /// seconds component and a nanoseconds component.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct BuiltinInterfacesDuration {
        /// Seconds component, range is valid over any possible int32 value.
        pub sec: i32,
        /// Nanoseconds component in the range of [0, 10e9).
        pub nanosec: u32,
    }
    /// Time indicates a specific point in time, relative to a clock's 0 point.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct BuiltinInterfacesTime {
        /// The seconds component, valid over all int32 values.
        pub sec: i32,
        /// The nanoseconds component, valid in the range [0, 10e9).
        pub nanosec: u32,
    }
    /// This represents the transform between two coordinate frames in free space.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsTransform {
        pub translation: GeometryMsgsVector3,
        pub rotation: GeometryMsgsQuaternion,
    }
    /// This contains the position of a point in free space
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoint {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    /// A representation of pose in free space, composed of position and orientation.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPose {
        pub position: GeometryMsgsPoint,
        pub orientation: GeometryMsgsQuaternion,
    }
    /// A Pose with reference coordinate frame and timestamp
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseStamped {
        pub header: StdMsgsHeader,
        pub pose: GeometryMsgsPose,
    }
    /// This represents a pose in free space with uncertainty.
    #[derive(Copy, Clone, Debug, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseWithCovariance {
        pub pose: GeometryMsgsPose,
        /// Row-major representation of the 6x6 covariance matrix
        ///
        /// The orientation parameters use a fixed-axis representation.
        ///
        /// In order, the parameters are:
        ///
        /// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        pub covariance: [f64; 36],
    }
    /// This expresses an estimated pose with a reference coordinate frame and timestamp
    #[derive(Clone, Debug, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseWithCovarianceStamped {
        pub header: StdMsgsHeader,
        pub pose: GeometryMsgsPoseWithCovariance,
    }
    /// This represents an orientation in free space in quaternion form.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsQuaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }
    /// This represents a vector in free space.
    ///
    /// This is semantically different than a point.
    ///
    /// A vector is always anchored at the origin.
    ///
    /// When a transform is applied to a vector, only the rotational component is applied.
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsVector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    /// Standard metadata for higher-level stamped data types.
    /// This is generally used to communicate timestamped data
    /// in a particular coordinate frame.
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct StdMsgsHeader {
        /// Two-integer timestamp that is expressed as seconds and nanoseconds.
        pub stamp: BuiltinInterfacesTime,
        /// Transform frame with which this data is associated.
        pub frame_id: String,
    }
}
