pub mod configs;
pub mod default_config;
pub mod default_msg;
pub mod msgs;

use cxx::UniquePtr;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};
use zenoh_flow::zenoh_flow_derive::ZFState;

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    unsafe extern "C++" {
        type NativeNode;
        fn init() -> UniquePtr<NativeNode>;
    }

    // unsafe extern "C++" {
    //     type GoalPose;
    //     fn goal_pose_init() -> UniquePtr<GoalPose>;
    //     fn goal_pose_get_goal_pose(node: &mut UniquePtr<GoalPose>) -> GeometryMsgsPoseStamped;
    //     fn goal_pose_is_new(node: &mut UniquePtr<GoalPose>) -> bool;
    // }
    // unsafe extern "C++" {
    //     type OsmMapLoader;
    //     fn osm_map_loader_init(cfg: &CfgOsmMapLoader) -> UniquePtr<OsmMapLoader>;
    // }
    // unsafe extern "C++" {
    //     type PcdMapLoader;
    //     fn pcd_map_loader_init(cfg: &CfgPcdMapLoader) -> UniquePtr<PcdMapLoader>;
    // }
    // unsafe extern "C++" {
    //     type GlobalPlanner;
    //     fn global_planner_init() -> UniquePtr<GlobalPlanner>;
    //     fn global_planner_set_current_pose(
    //         node: &mut UniquePtr<GlobalPlanner>,
    //         msg: &AutowareAutoMsgsVehicleKinematicState,
    //     );
    //     fn global_planner_set_goal_pose(
    //         node: &mut UniquePtr<GlobalPlanner>,
    //         msg: &GeometryMsgsPoseStamped,
    //     );
    //     fn global_planner_get_route(
    //         node: &mut UniquePtr<GlobalPlanner>,
    //     ) -> AutowareAutoMsgsHadmapRoute;
    // }
    // unsafe extern "C++" {
    //     type LocalPlanner;
    //     fn local_planner_init(cfg: &CfgLocalPlanner) -> UniquePtr<LocalPlanner>;
    //     fn local_planner_set_route(
    //         node: &mut UniquePtr<LocalPlanner>,
    //         msg: &AutowareAutoMsgsHadmapRoute,
    //     );
    //     fn local_planner_set_kinematic_state(
    //         node: &mut UniquePtr<LocalPlanner>,
    //         msg: &AutowareAutoMsgsVehicleKinematicState,
    //     );
    //     fn local_planner_set_state_report(
    //         node: &mut UniquePtr<LocalPlanner>,
    //         msg: &AutowareAutoMsgsVehicleStateReport,
    //     );
    //     fn local_planner_get_trajectory(
    //         node: &mut UniquePtr<LocalPlanner>,
    //     ) -> AutowareAutoMsgsTrajectory;
    //     fn local_planner_get_state_cmd(
    //         node: &mut UniquePtr<LocalPlanner>,
    //     ) -> AutowareAutoMsgsVehicleStateCommand;
    // }
    // unsafe extern "C++" {
    //     type LanePlanner;
    //     fn lane_planner_init(cfg: &CfgLanePlanner) -> UniquePtr<LanePlanner>;
    // }
    // unsafe extern "C++" {
    //     type ParkingPlanner;
    //     fn parking_planner_init(cfg: &CfgParkingPlanner) -> UniquePtr<ParkingPlanner>;
    // }
    // unsafe extern "C++" {
    //     type PurePursuit;
    //     fn pure_pursuit_init(cfg: &CfgPurePursuit) -> UniquePtr<PurePursuit>;
    //     fn pure_pursuit_set_trajectory(
    //         node: &mut UniquePtr<PurePursuit>,
    //         msg: &AutowareAutoMsgsTrajectory,
    //     );
    //     fn pure_pursuit_set_kinematic_state(
    //         node: &mut UniquePtr<PurePursuit>,
    //         msg: &AutowareAutoMsgsVehicleKinematicState,
    //     );
    //     fn pure_pursuit_get_control_cmd(
    //         node: &mut UniquePtr<PurePursuit>,
    //     ) -> AutowareAutoMsgsVehicleControlCommand;
    // }
    // unsafe extern "C++" {
    //     type Simulator;
    //     fn simulator_init(cfg: &CfgSimulator) -> UniquePtr<Simulator>;
    //     fn simulator_set_init_pose(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &GeometryMsgsPoseWithCovarianceStamped,
    //     );
    //     fn simulator_set_control_cmd(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &AutowareAutoMsgsVehicleControlCommand,
    //     );
    //     fn simulator_set_state_cmd(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &AutowareAutoMsgsVehicleStateCommand,
    //     );
    //     fn simulator_get_kinematic_state(
    //         node: &mut UniquePtr<Simulator>,
    //     ) -> AutowareAutoMsgsVehicleKinematicState;
    //     fn simulator_get_state_report(
    //         node: &mut UniquePtr<Simulator>,
    //     ) -> AutowareAutoMsgsVehicleStateReport;
    //     fn simulator_update(node: &mut UniquePtr<Simulator>);
    // }
}

unsafe impl Send for ffi::NativeNode {}
unsafe impl Sync for ffi::NativeNode {}

impl Debug for ffi::NativeNode {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode>,
}
