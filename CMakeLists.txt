# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drone)

target_sources(app PRIVATE
    src/main.c
    src/drone_receiver.c
    src/drone_mp9250.c
    src/drone_mc_controller.c
)


