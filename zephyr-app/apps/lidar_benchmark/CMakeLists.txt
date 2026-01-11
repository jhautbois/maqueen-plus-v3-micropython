# LIDAR Benchmark Application
#
# Measures real hardware performance of SEN0628 sensor
#
# Build from zephyr-app directory:
#   west build -b bbc_microbit_v2 apps/lidar_benchmark -p
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Path to parent zephyr-app directory
set(ZEPHYR_APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Add local DTS bindings
list(APPEND DTS_ROOT ${ZEPHYR_APP_ROOT})

# Add extra Kconfig directories
set(KCONFIG_ROOT ${ZEPHYR_APP_ROOT}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(lidar_benchmark)

# Application sources
target_sources(app PRIVATE src/main.c)

# Include driver directory
target_include_directories(app PRIVATE
    ${ZEPHYR_APP_ROOT}/drivers/sen0628
)

# Add driver sources directly (until we have proper module setup)
target_sources(app PRIVATE
    ${ZEPHYR_APP_ROOT}/drivers/sen0628/sen0628.c
)
