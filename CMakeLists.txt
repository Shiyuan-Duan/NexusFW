# CMakeLists.txt at the repository root
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NexusFW)

# ── Public headers ──
# One top‑level include directory is enough;   #include <ble/ble.h>
# is resolved as "${PROJECT_SOURCE_DIR}/include/ble/ble.h".
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/include/driver
)

# ── Sources ──
# Collect every *.c under src/ (recursively) and re‑configure
# whenever a matching file is added or removed.
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

target_sources(app PRIVATE ${APP_SOURCES})
