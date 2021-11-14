#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_global_planner.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode
            {
            };
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void set_current_pose(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_goal_pose(std::unique_ptr<NativeNode> &, const GeometryMsgsPoseStamped &);
        }
    }
}
