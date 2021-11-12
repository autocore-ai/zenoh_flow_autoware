use async_trait::async_trait;
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{
    lane_planner_init, LanePlanner, LanePlannerConfig, CfgLanePlanner, Vehicle, GaussianSmoother
};
use ffi::common_type::{ ZFUsize };
use std::sync::Arc;
use zenoh_flow::{
    export_source, zenoh_flow_derive::ZFState, Configuration, Context, Data, Node, Source, State,
    ZFError, ZFResult, async_std
};

#[derive(Debug, ZFState)]
pub struct LanePlannerSource;
unsafe impl Send for LanePlannerSource {}
unsafe impl Sync for LanePlannerSource {}

#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<LanePlanner>,
}

impl Node for LanePlannerSource {
    fn initialize(&self, configuration: &Option<Configuration>) -> ZFResult<State> {
        let cfg = match configuration {
            Some(config) => {
                if let (
                    Some(heading_weight),
                    Some(trajectory_resolution),
                    Some(standard_deviation),
                    Some(kernel_size),
                    Some(cg_to_front_m),
                    Some(cg_to_rear_m),
                    Some(front_corner_stiffness),
                    Some(rear_corner_stiffness),
                    Some(mass_kg),
                    Some(yaw_inertia_kgm2),
                    Some(width_m),
                    Some(front_overhang_m),
                    Some(rear_overhang_m),
                ) = (
                    config.get("heading_weight"),
                    config.get("trajectory_resolution"),
                    config.get("standard_deviation"),
                    config.get("kernel_size"),
                    config.get("cg_to_front_m"),
                    config.get("cg_to_rear_m"),
                    config.get("front_corner_stiffness"),
                    config.get("rear_corner_stiffness"),
                    config.get("mass_kg"),
                    config.get("yaw_inertia_kgm2"),
                    config.get("width_m"),
                    config.get("front_overhang_m"),
                    config.get("rear_overhang_m"),
                ) {

                    let lane_planner_cfg = LanePlannerConfig {
                        trajectory_resolution: trajectory_resolution.as_f64().unwrap(),
                    };
                    let vehicle_cfg = Vehicle {
                        cg_to_front_m: cg_to_front_m.as_f64().unwrap(),
                        cg_to_rear_m: cg_to_rear_m.as_f64().unwrap(),
                        front_corner_stiffness: front_corner_stiffness.as_f64().unwrap(),
                        rear_corner_stiffness: rear_corner_stiffness.as_f64().unwrap(),
                        mass_kg: mass_kg.as_f64().unwrap(),
                        yaw_inertia_kgm2: yaw_inertia_kgm2.as_f64().unwrap(),
                        width_m: width_m.as_f64().unwrap(),
                        front_overhang_m: front_overhang_m.as_f64().unwrap(),
                        rear_overhang_m: rear_overhang_m.as_f64().unwrap(),
                    };
                    let gaussian_smoother_cfg =  GaussianSmoother {
                        standard_deviation: standard_deviation.as_f64().unwrap(),
                        kernel_size: kernel_size.as_i64().unwrap(),
                    };
                    let cfg = CfgLanePlanner {
                        heading_weight: heading_weight.as_f64().unwrap(),
                        lane_planner: lane_planner_cfg,
                        vehicle: vehicle_cfg,
                        gaussian_smoother: gaussian_smoother_cfg,
                    };
                    log::debug!("Configuration of LanePlannerSource: {:?}", cfg);
                    Ok(cfg)
                } else {
                    Err(ZFError::MissingConfiguration)
                }
            }
            None => Err(ZFError::InvalidData(String::from("Configuration of LanePlannerSource is None")))
        }?;

        let ptr = lane_planner_init(&cfg);
        Ok(State::from(Instance { ptr }))
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Source for LanePlannerSource {
    async fn run(&self, _context: &mut Context, _dyn_state: &mut State) -> ZFResult<Data> {
        log::debug!("zenoh flow lane planner running ...");
        async_std::task::sleep(std::time::Duration::from_secs(1)).await;
        Ok(Data::from::<ZFUsize>(ZFUsize(1)))
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    env_logger::init();
    Ok(Arc::new(LanePlannerSource) as Arc<dyn Source>)
}
