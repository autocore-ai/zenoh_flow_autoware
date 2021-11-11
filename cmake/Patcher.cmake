message(STATUS "Patching ${HEADER}")

file(READ ${HEADER} header_unpatched)

string(REPLACE "#include \"zenoh_flow_init_pose/zenoh_flow_init_pose.hpp\"" ""
  header_patched "${header_unpatched}")

file(WRITE ${HEADER} "${header_patched}")

file(READ ${HEADER} header_unpatched)

string(REPLACE "#include \"zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp\"" ""
  header_patched "${header_unpatched}")

file(WRITE ${HEADER} "${header_patched}")

file(READ ${HEADER} header_unpatched)

string(REPLACE "      using InitPose = ::zenoh_flow::autoware_auto::ffi::InitPose;" ""
  header_patched "${header_unpatched}")

file(WRITE ${HEADER} "${header_patched}")

file(READ ${HEADER} header_unpatched)

string(REPLACE "      using GoalPose = ::zenoh_flow::autoware_auto::ffi::GoalPose;" ""
  header_patched "${header_unpatched}")

file(WRITE ${HEADER} "${header_patched}")

message(STATUS "Patching ${HEADER} - done")
