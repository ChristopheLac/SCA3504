# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "sca3504.overlay")
#project("SCA3504" VERSION 0.1)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(modbus-rtu-client)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

