message(STATUS "Patching ${TARGET}")

function(AddInclude)
    foreach(arg IN LISTS ARGN)
        execute_process(COMMAND sed -i "1i #include <${arg}/${arg}.hpp>" ${TARGET})
    endforeach()
endfunction(AddInclude)

AddInclude(
    zenoh_flow_global_planner
    zenoh_flow_goal_pose
    zenoh_flow_init_pose
    zenoh_flow_lane_planner
    zenoh_flow_local_planner
    zenoh_flow_osm_map_loader
    zenoh_flow_parking_planner
    zenoh_flow_pcd_map_loader
    zenoh_flow_pure_pursuit
    zenoh_flow_simulator
)

message(STATUS "Patching ${TARGET} - done")
