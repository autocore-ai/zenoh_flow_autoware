message(STATUS "Patching ${TARGET}")

function(RemoveUsing)
    foreach(arg IN LISTS ARGN)
        file(READ ${TARGET} unpatched)
        string(REPLACE "using ${arg} = ::zenoh_flow::autoware_auto::ffi::${arg};" "" patched "${unpatched}")
        file(WRITE ${TARGET} "${patched}")
    endforeach()
endfunction(RemoveUsing)

RemoveUsing(
    GlobalPlanner
    GoalPose
    InitPose
    LanePlanner
    LocalPlanner
    OsmMapLoader
    ParkingPlanner
    PcdMapLoader
    PurePursuit
    Simulator
)

message(STATUS "Patching ${TARGET} - done")
