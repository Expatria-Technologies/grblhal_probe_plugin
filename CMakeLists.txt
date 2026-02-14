add_library(grblhal_probe_plugin INTERFACE)

target_sources(grblhal_probe_plugin INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/probe_plugin.c
)

target_include_directories(grblhal_probe_plugin INTERFACE ${CMAKE_CURRENT_LIST_DIR})
