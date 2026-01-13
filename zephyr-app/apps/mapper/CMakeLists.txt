# SPDX-License-Identifier: Apache-2.0
#
# Mapper Application - Autonomous exploration with occupancy grid mapping

cmake_minimum_required(VERSION 3.20.0)

# Path to parent zephyr-app directory
set(ZEPHYR_APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Add local DTS bindings
list(APPEND DTS_ROOT ${ZEPHYR_APP_ROOT})

# Add extra Kconfig directories (must be before find_package)
set(KCONFIG_ROOT ${ZEPHYR_APP_ROOT}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mapper)

# Include paths
target_include_directories(app PRIVATE
    ${ZEPHYR_APP_ROOT}/src
    ${ZEPHYR_APP_ROOT}/drivers/sen0628
)

# Application sources
target_sources(app PRIVATE
    src/main.c
)

# Shared modules from ../../src
target_sources(app PRIVATE
    ${ZEPHYR_APP_ROOT}/src/maqueen.c
    ${ZEPHYR_APP_ROOT}/src/pid_control.c
    ${ZEPHYR_APP_ROOT}/src/odometry.c
    ${ZEPHYR_APP_ROOT}/src/occupancy_grid.c
    ${ZEPHYR_APP_ROOT}/src/scanner_360.c
    ${ZEPHYR_APP_ROOT}/src/frontier_nav.c
    ${ZEPHYR_APP_ROOT}/src/map_storage.c
    ${ZEPHYR_APP_ROOT}/src/map_shell.c
)

# SEN0628 LIDAR driver
target_sources(app PRIVATE
    ${ZEPHYR_APP_ROOT}/drivers/sen0628/sen0628.c
)
