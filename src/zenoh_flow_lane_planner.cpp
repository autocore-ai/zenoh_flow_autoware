#include <zenoh_flow_lane_planner/zenoh_flow_lane_planner.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode::NativeNode() {}
            NativeNode::NativeNode(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(rclcpp::Parameter("heading_weight", cfg.heading_weight));
                paramters.push_back(rclcpp::Parameter("lane_planner.trajectory_resolution", cfg.lane_planner.trajectory_resolution));
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_front_m", cfg.vehicle.cg_to_front_m));
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_rear_m", cfg.vehicle.cg_to_rear_m));
                paramters.push_back(rclcpp::Parameter("vehicle.front_corner_stiffness", cfg.vehicle.front_corner_stiffness));
                paramters.push_back(rclcpp::Parameter("vehicle.rear_corner_stiffness", cfg.vehicle.rear_corner_stiffness));
                paramters.push_back(rclcpp::Parameter("vehicle.mass_kg", cfg.vehicle.mass_kg));
                paramters.push_back(rclcpp::Parameter("vehicle.yaw_inertia_kgm2", cfg.vehicle.yaw_inertia_kgm2));
                paramters.push_back(rclcpp::Parameter("vehicle.width_m", cfg.vehicle.width_m));
                paramters.push_back(rclcpp::Parameter("vehicle.front_overhang_m", cfg.vehicle.front_overhang_m));
                paramters.push_back(rclcpp::Parameter("vehicle.rear_overhang_m", cfg.vehicle.rear_overhang_m));
                paramters.push_back(rclcpp::Parameter("gaussian_smoother.standard_deviation", cfg.gaussian_smoother.standard_deviation));
                paramters.push_back(rclcpp::Parameter("gaussian_smoother.kernel_size", cfg.gaussian_smoother.kernel_size));
                options.parameter_overrides(paramters);
                ptr = std::make_shared<autoware::lane_planner_nodes::LanePlannerNode>(options);
                std::thread{std::bind(&NativeNode::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void NativeNode::spin()
            {
                while (rclcpp::ok())
                {
                    rclcpp::spin_some(ptr);
                }
            }

            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }

            std::unique_ptr<NativeNode> init(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode>(cfg);
            }

            std::unique_ptr<NativeNode> init_null_config()
            {
                return std::make_unique<NativeNode>();
            }
        }
    }
}