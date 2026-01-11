# Proximity LED Visualization Application
#
# Displays obstacle proximity using headlights and underglow LEDs.
# Color indicates distance: green=safe, yellow=caution, red=danger.
#
# Build:
#   west build -b bbc_microbit_v2 apps/proximity_leds -p
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

list(APPEND DTS_ROOT ${ZEPHYR_APP_ROOT})
set(KCONFIG_ROOT ${ZEPHYR_APP_ROOT}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(proximity_leds)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE
    ${ZEPHYR_APP_ROOT}/drivers/sen0628
    ${ZEPHYR_APP_ROOT}/src
)

target_sources(app PRIVATE
    ${ZEPHYR_APP_ROOT}/drivers/sen0628/sen0628.c
    ${ZEPHYR_APP_ROOT}/src/maqueen.c
)
