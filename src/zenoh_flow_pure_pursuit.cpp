#include <zenoh_flow_pure_pursuit/zenoh_flow_pure_pursuit.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            PurePursuit::PurePursuit(const CfgPurePursuit &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                auto config = autoware::motion::control::pure_pursuit::Config(
                    cfg.minimum_lookahead_distance,
                    cfg.maximum_lookahead_distance,
                    cfg.speed_to_lookahead_ratio,
                    cfg.is_interpolate_lookahead_point,
                    cfg.is_delay_compensation,
                    cfg.emergency_stop_distance,
                    cfg.speed_thres_traveling_direction,
                    cfg.distance_front_rear_wheel);
                ptr = std::make_shared<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode>(
                    "pure_pursuit_node", config, "", autocore::NodeType::ZenohFlow);
            }
            AutowareAutoMsgsVehicleControlCommand PurePursuit::GetControlCmd()
            {
                return Convert(ptr->GetVehicleCmd());
            }
            void PurePursuit::SetTrajectory(const AutowareAutoMsgsTrajectory &msg)
            {
                ptr->SetTrajectory(Convert(msg));
                rclcpp::spin_some(this->ptr);
            }
            void PurePursuit::SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                ptr->SetKinematicState(Convert(msg));
                rclcpp::spin_some(this->ptr);
            }
            AutowareAutoMsgsVehicleControlCommand pure_pursuit_get_control_cmd(std::unique_ptr<PurePursuit> &);
            std::unique_ptr<PurePursuit> pure_pursuit_init(const CfgPurePursuit &);
            void pure_pursuit_set_kinematic_state(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsVehicleKinematicState &);
            void pure_pursuit_set_trajectory(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}
