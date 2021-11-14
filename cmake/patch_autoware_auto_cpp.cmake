message(STATUS "Patching ${TARGET}")

function(AddInclude)
    foreach(arg IN LISTS ARGN)
        execute_process(COMMAND sed -i "1i #ifdef build_${arg}\\n#include <${arg}/${arg}.hpp>\\n#endif" ${TARGET})
    endforeach()
endfunction(AddInclude)

AddInclude(
    zenoh_flow_global_planner
    zenoh_flow_goal_pose
    zenoh_flow_init_pose
    zenoh_flow_lane_planner
    zenoh_flow_osm_map_loader
    zenoh_flow_parking_planner
    zenoh_flow_pcd_map_loader
    )

message(STATUS "Patching ${TARGET} - done")
