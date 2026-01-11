# SEN0628 ToF LIDAR Driver

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SEN0628 sen0628.c)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
