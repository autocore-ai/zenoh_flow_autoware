# Copyright 2021 The AutoCore.AI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(zenoh_flow_autoware)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_program(CXXBRIDGE cxxbridge REQUIRED PATHS $ENV{HOME}/.cargo/bin)

message(STATUS "Using cxxbridge: ${CXXBRIDGE}")

macro(GenCommonFFI rust_src_path)

set(cxx_file_file ${ARGV1})

if("${cxx_file_file} " STREQUAL " ")
  STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" cxx_file_file ${rust_src_path})
endif()

message(STATUS "Start GenCommonFFI : ${cxx_file_file}")

set(cxxbridge_header_out ${CMAKE_BINARY_DIR}/${cxx_file_file}.hpp)
set(cxxbridge_source_out ${CMAKE_BINARY_DIR}/${cxx_file_file}.cpp)

add_custom_target(CommonFFI_${cxx_file_file} ALL
  DEPENDS ${cxxbridge_header_out} ${cxxbridge_source_out})

add_custom_command(
  OUTPUT ${cxxbridge_header_out} ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${rust_src_path} --output ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${rust_src_path} --header --output ${cxxbridge_header_out}
  COMMAND ${CMAKE_COMMAND} -DTARGET=${cxxbridge_header_out} -P ${CMAKE_SOURCE_DIR}/cmake/patch_${cxx_file_file}_hpp.cmake
  COMMAND ${CMAKE_COMMAND} -DTARGET=${cxxbridge_source_out} -P ${CMAKE_SOURCE_DIR}/cmake/patch_${cxx_file_file}_cpp.cmake
  COMMENT "Generating Common FFI: ${cxx_file_file}"
)

message(STATUS "Finish GenCommonFFI : ${cxx_file_file}")

endmacro(GenCommonFFI)

macro(GenNodeFFI node_name)

message(STATUS "Start GenNodeFFI : ${node_name}")

set(cxxbridge_in ${CMAKE_SOURCE_DIR}/${node_name}/src/ffi.rs)
set(cxxbridge_header_out ${CMAKE_BINARY_DIR}/${node_name}.hpp)
set(cxxbridge_source_out ${CMAKE_BINARY_DIR}/${node_name}.cpp)

add_custom_target(NodeFFI_${node_name} ALL
  DEPENDS CommonFFI_autoware_auto CommonFFI_msgs ${cxxbridge_header_out} ${cxxbridge_source_out})

add_custom_command(
  OUTPUT ${cxxbridge_header_out} ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --output ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --header --output ${cxxbridge_header_out}
  COMMENT "Generating Node FFI: ${node_name}"
)
message(STATUS "Finish GenNodeFFI : ${node_name}")
  
endmacro(GenNodeFFI node_name)

macro(GenNode node_name)

GenNodeFFI(${node_name})

set(rust_lib ${CMAKE_SOURCE_DIR}/target/release/lib${node_name}.a)

message(STATUS "Start GenNode : ${node_name}")

add_custom_target(WrapperLib_${node_name} ALL DEPENDS ${rust_lib})
add_custom_command(
  OUTPUT ${rust_lib}
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  DEPENDS NodeFFI_${node_name}
  COMMENT "Generating Rust library: ${node_name}"
  )

message(STATUS "Finish WrapperLib_${node_name}")

add_library(CxxWrapper_${node_name} STATIC IMPORTED GLOBAL)
add_dependencies(CxxWrapper_${node_name} WrapperLib_${node_name})
set_target_properties(CxxWrapper_${node_name}
  PROPERTIES
  IMPORTED_LOCATION ${rust_lib}
)

message(STATUS "Finish CxxWrapper_${node_name}")

ament_auto_add_library(${node_name} SHARED
  ${CMAKE_BINARY_DIR}/autoware_auto.cpp
  ${CMAKE_BINARY_DIR}/msgs.cpp
  ${CMAKE_BINARY_DIR}/${node_name}.cpp
  src/zenoh_flow_msg_convert.cpp
  src/${node_name}.cpp
  ${ARGV1}
  ${ARGV2}
  ${ARGV3}
  ${ARGV4}
)
target_compile_definitions(${node_name} PRIVATE build_${node_name})
add_dependencies(${node_name} CxxWrapper_${node_name})
target_link_libraries(${node_name} "-Wl,--whole-archive" CxxWrapper_${node_name} "-Wl,--no-whole-archive")

message(STATUS "Finish GenNode : ${node_name}")

endmacro(GenNode)

include_directories(${CMAKE_BINARY_DIR})

GenCommonFFI(${CMAKE_SOURCE_DIR}/autoware_auto/src/lib.rs autoware_auto)
GenCommonFFI(${CMAKE_SOURCE_DIR}/autoware_auto/src/msgs.rs)

GenNode(zenoh_flow_global_planner)
GenNode(zenoh_flow_goal_pose src/goal_pose_receiver.cpp)
GenNode(zenoh_flow_init_pose src/init_pose_receiver.cpp)
GenNode(zenoh_flow_lane_planner)
GenNode(zenoh_flow_local_planner)
GenNode(zenoh_flow_osm_map_loader)
GenNode(zenoh_flow_parking_planner)
GenNode(zenoh_flow_pcd_map_loader)
GenNode(zenoh_flow_pure_pursuit)
GenNode(zenoh_flow_simulator)

ament_auto_package()
