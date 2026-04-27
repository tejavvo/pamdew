FILE(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/src/*.c
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        "."
        "${CMAKE_SOURCE_DIR}/lib/calibration"
        "${CMAKE_SOURCE_DIR}/lib/pan_tompkins"
        "${CMAKE_SOURCE_DIR}/lib/wifi_tx"
)
