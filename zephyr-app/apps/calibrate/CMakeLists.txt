# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr (sets PROJECT_BINARY_DIR, PROJECT_SOURCE_DIR...)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(calibrate VERSION 1.0.0)

# Application sources
target_sources(app PRIVATE
    src/main.c
    src/calib_tests.c
    src/calib_compute.c
    src/calib_shell.c
)

# Include shared sources from parent
zephyr_library_sources(
    ../../src/maqueen.c
    ../../src/odometry.c
    ../../src/map_storage.c
)

# Include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)
