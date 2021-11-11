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

set(rust_code_file autoware_auto)
set(cxxbridge_in         ${CMAKE_SOURCE_DIR}/ffi/src/${rust_code_file}.rs)
set(cxxbridge_header_out ${CMAKE_BINARY_DIR}/${rust_code_file}.hpp)
set(cxxbridge_source_out ${CMAKE_BINARY_DIR}/${rust_code_file}.cpp)

add_custom_target(CxxBridge ALL
  DEPENDS ${cxxbridge_header_out} ${cxxbridge_source_out})

add_custom_command(
  OUTPUT ${cxxbridge_header_out} ${cxxbridge_source_out}
  COMMAND find ${CMAKE_BINARY_DIR} -name "${rust_code_file}.*" |xargs rm -rf
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --output ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --header --output ${cxxbridge_header_out}
  COMMAND ${CMAKE_COMMAND} -DHEADER=${cxxbridge_header_out} -P ${CMAKE_SOURCE_DIR}/cmake/Patcher.cmake
  COMMENT "Generating CXX bridge:"
)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

macro(GenLib)

set(cxxwrapper_lib ${CMAKE_SOURCE_DIR}/target/release/lib${ARGV0}.a)

add_custom_target(WrapperLib_${ARGV0} ALL DEPENDS ${cxxwrapper_lib})
add_custom_command(
  OUTPUT ${cxxwrapper_lib}
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  DEPENDS ${cxxbridge_in}
  COMMENT "Generating Rust library:"
  )

add_library(CxxWrapper_${ARGV0} STATIC IMPORTED GLOBAL)
add_dependencies(CxxWrapper_${ARGV0} WrapperLib_${ARGV0})
set_target_properties(CxxWrapper_${ARGV0}
  PROPERTIES
  IMPORTED_LOCATION ${cxxwrapper_lib}
)

include_directories(${CMAKE_BINARY_DIR})

ament_auto_add_library(${ARGV0} SHARED ${cxxbridge_source_out} src/zenoh_flow_msg_convert.cpp src/${ARGV0}.cpp ${ARGV1} ${ARGV2} ${ARGV3} ${ARGV4})
add_dependencies(${ARGV0} CxxBridge)
target_link_libraries(${ARGV0} "-Wl,--whole-archive" CxxWrapper_${ARGV0} "-Wl,--no-whole-archive")

endmacro(GenLib)

GenLib(zenoh_flow_init_pose)
GenLib(zenoh_flow_goal_pose)

ament_auto_package()
