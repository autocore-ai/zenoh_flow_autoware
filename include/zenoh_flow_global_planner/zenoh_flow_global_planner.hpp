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
            class NativeNode_global_planner
            {
            public:
                NativeNode_global_planner(const NativeConfig &);
                AutowareAutoMsgsHadmapRoute GetRoute();
                void SetCurrentPose(const AutowareAutoMsgsVehicleKinematicState &);
                void SetGoalPose(const GeometryMsgsPoseStamped &);

            private:
                std::shared_ptr<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode> ptr;
            };
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<NativeNode_global_planner> &);
            std::unique_ptr<NativeNode_global_planner> init_global_planner(const NativeConfig &);
            void set_current_pose(std::unique_ptr<NativeNode_global_planner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_goal_pose(std::unique_ptr<NativeNode_global_planner> &, const GeometryMsgsPoseStamped &);
        }
    }
}
