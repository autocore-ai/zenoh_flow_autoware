#pragma once
#include <msgs.hpp>
#include <zenoh_flow_global_planner.hpp>
#include <lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode
            {
            public:
                NativeNode();
                NativeNode(const NativeConfig &);
                AutowareAutoMsgsHadmapRoute GetRoute();
                void SetCurrentPose(const AutowareAutoMsgsVehicleKinematicState &);
                void SetGoalPose(const GeometryMsgsPoseStamped &);

            private:
                std::shared_ptr<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode> ptr;
            };
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void set_current_pose(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_goal_pose(std::unique_ptr<NativeNode> &, const GeometryMsgsPoseStamped &);
        }
    }
}
