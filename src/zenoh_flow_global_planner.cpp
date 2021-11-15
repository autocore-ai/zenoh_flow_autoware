#include <zenoh_flow_global_planner/zenoh_flow_global_planner.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>
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
                std::cout << "Lanelet2GlobalPlannerNode" << std::endl;
                ptr = std::make_shared<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode>(
                    options, autocore::NodeType::ZenohFlow);
            }

            void NativeNode::SetCurrentPose(const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                ptr->SetCurrentPose(Convert(msg));
            }
            void NativeNode::SetGoalPose(const GeometryMsgsPoseStamped &msg)
            {
                ptr->SetGoalPose(Convert(msg));
            }
            AutowareAutoMsgsHadmapRoute NativeNode::GetRoute() { return Convert(ptr->GetRoute()); }
            std::unique_ptr<NativeNode> init(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode>(cfg);
            }
            std::unique_ptr<NativeNode> init_null_config() { return std::make_unique<NativeNode>(); }
            void set_current_pose(std::unique_ptr<NativeNode> &node, const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                node->SetCurrentPose(msg);
            }
            void set_goal_pose(std::unique_ptr<NativeNode> &node, const GeometryMsgsPoseStamped &msg)
            {
                node->SetGoalPose(msg);
            }
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<NativeNode> &node)
            {
                return node->GetRoute();
            }
        } // namespace ffi

    } // namespace autoware_auto

} // namespace zenoh_flow