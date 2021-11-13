use crate::autoware_auto::msgs::ffi::*;

impl Default for GeometryMsgsPoseWithCovariance {
    fn default() -> Self {
        GeometryMsgsPoseWithCovariance {
            pose: GeometryMsgsPose::default(),
            covariance: [0.0; 36],
        }
    }
}
