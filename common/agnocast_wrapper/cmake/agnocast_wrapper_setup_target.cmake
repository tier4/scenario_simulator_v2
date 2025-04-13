# Copyright 2015 TIER IV, Inc. All rights reserved.
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

# based on https://github.com/veqcc/autoware.universe/blob/252ae60788a8388585780a6b1935a5682c688464/common/autoware_agnocast_wrapper/cmake/agnocast_wrapper_config_extras.cmake#L1-L8  // NOLINT
function(agnocast_wrapper_setup target)
  if(DEFINED ENV{ENABLE_AGNOCAST_SIMULATOR} AND "$ENV{ENABLE_AGNOCAST_SIMULATOR}" STREQUAL "1")
    message(WARNING "agnocast_wrapper_setup: Defining USE_AGNOCAST_ENABLED for target ${target}")
    target_compile_definitions(${target} PUBLIC USE_AGNOCAST_ENABLED)
  else()
    message(WARNING "agnocast_wrapper_setup: Not defining USE_AGNOCAST_ENABLED for target ${target}")
  endif()
endfunction()
