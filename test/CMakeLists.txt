# test/CMakeLists.txt — small command-line dump / diagnostic tools.
#
# Each tool links prad2dec (via add_prad_tool).  LINKS adds extra private
# dependencies; SRC overrides the default source path (test/<name>.cpp).

# Tools that need nlohmann/json (for --daq-config parsing).
foreach(_t evio_dump ped_calc ts_dump livetime)
    add_prad_tool(${_t} LINKS nlohmann_json::nlohmann_json)
endforeach()

# gem_dump additionally needs prad2det for clustering.
add_prad_tool(gem_dump LINKS prad2det nlohmann_json::nlohmann_json)

# ET-only tools (compiled only when WITH_ET is on in prad2dec).
if(WITH_ET)
    add_prad_tool(evc_test     SRC test_main.cpp)
    add_prad_tool(et_feeder)
    add_prad_tool(evchan_test)
endif()
