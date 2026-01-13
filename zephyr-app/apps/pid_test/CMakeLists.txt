# SPDX-License-Identifier: Apache-2.0
#
# PID Test Application - Test hardware PID control

cmake_minimum_required(VERSION 3.20.0)

# Path to parent zephyr-app directory
set(ZEPHYR_APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Add local DTS bindings
list(APPEND DTS_ROOT ${ZEPHYR_APP_ROOT})

# Add extra Kconfig directories
set(KCONFIG_ROOT ${ZEPHYR_APP_ROOT}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pid_test)

# Include paths
target_include_directories(app PRIVATE
    ${ZEPHYR_APP_ROOT}/src
)

# Application sources
target_sources(app PRIVATE
    src/main.c
)

# Shared modules
target_sources(app PRIVATE
    ${ZEPHYR_APP_ROOT}/src/maqueen.c
    ${ZEPHYR_APP_ROOT}/src/pid_control.c
    ${ZEPHYR_APP_ROOT}/src/odometry.c
)
