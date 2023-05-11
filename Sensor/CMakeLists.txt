cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scu)

include_directories(
                    scu_sensors
                    src
)


FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE 
                        src/main.c
                        scu_sensors/scu_sensors.c
)
