# based on https://github.com/veqcc/autoware.universe/blob/252ae60788a8388585780a6b1935a5682c688464/common/autoware_agnocast_wrapper/cmake/agnocast_wrapper_config_extras.cmake#L1-L8
function(agnocast_wrapper_setup_target target)
  if(DEFINED ENV{ENABLE_AGNOCAST_SIMULATOR} AND "$ENV{ENABLE_AGNOCAST_SIMULATOR}" STREQUAL "1")
    message(WARNING "agnocast_wrapper_setup_target: Defining USE_AGNOCAST_ENABLED for target ${target}")
    target_compile_definitions(${target} PUBLIC USE_AGNOCAST_ENABLED)
  else()
    message(WARNING "agnocast_wrapper_setup_target: Not defining USE_AGNOCAST_ENABLED for target ${target}")
  endif()
endfunction()
