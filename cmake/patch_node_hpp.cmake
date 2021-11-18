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

message(STATUS "Patching ${TARGET}")

STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" node_name ${TARGET})

string(REPLACE "zenoh_flow" "NativeNode" class_name "${node_name}")

file(READ ${TARGET} header_unpatched)

string(REPLACE "using ${class_name} = ::zenoh_flow::autoware_auto::ffi::${class_name};" "" header_patched "${header_unpatched}")

file(WRITE ${TARGET} "${header_patched}")

message(STATUS "Patching ${TARGET} - done")
