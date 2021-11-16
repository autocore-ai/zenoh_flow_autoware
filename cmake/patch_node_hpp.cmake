message(STATUS "Patching ${TARGET}")

STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" node_name ${TARGET})

string(REPLACE "zenoh_flow" "NativeNode" class_name "${node_name}")

file(READ ${TARGET} header_unpatched)

string(REPLACE "using ${class_name} = ::zenoh_flow::autoware_auto::ffi::${class_name};" "" header_patched "${header_unpatched}")

file(WRITE ${TARGET} "${header_patched}")

message(STATUS "Patching ${TARGET} - done")
