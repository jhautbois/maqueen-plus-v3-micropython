# Maqueen Plus V3 Zephyr Test Application

cmake_minimum_required(VERSION 3.20.0)

# Add local DTS bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(maqueen_test)

target_sources(app PRIVATE
    src/main.c
    src/maqueen.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
