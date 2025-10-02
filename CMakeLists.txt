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

# ── Sources (conditional compile per app) ──
# Keep this convenient: auto-include drivers; explicitly include app variant.

# Drivers (auto-include any new driver/*.c without editing this file)
file(GLOB DRIVER_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/*.c)

# Always include main; shared BLE advertising core is linked once
set(BASE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
)

# App-specific sources
if (CONFIG_APP_CACHEXIA)
  message(STATUS "Building Cachexia application")
  set(APP_SPECIFIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src/app/cachexia_app.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_cachexia_srv.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_adv_core.c)
elseif (CONFIG_APP_STIM_DEMO)
  message(STATUS "Building STIM_DEMO application")
  set(APP_SPECIFIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_ad5689_srv.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_advertise.c)
elseif (CONFIG_APP_DRUG_DELIVERY)
  message(STATUS "Building DRUG_DELIVERY application")
  set(APP_SPECIFIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src/app/drug_delivery_app.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_drug_delivery_srv.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_adv_core.c)
elseif (CONFIG_APP_GFET_MEASURE)
  message(STATUS "Building GFET_MEASURE application")
  set(APP_SPECIFIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src/app/gfet_app.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_gfet_srv.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_adv_core.c)
else()
  message(STATUS "No app variant selected; defaulting to STIM_DEMO")
  set(APP_SPECIFIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_ad5689_srv.c
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/ble_advertise.c)
endif()

target_sources(app PRIVATE ${BASE_SOURCES} ${DRIVER_SOURCES} ${APP_SPECIFIC})
