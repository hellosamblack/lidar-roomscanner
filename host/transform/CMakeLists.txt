cmake_minimum_required(VERSION 3.22)
project(roomscan_transform C)

# 53L9A1 reference package (read-only) providing the vl53l9-transform-c /
# media-object sources. Compiled in place; never copied or edited.
# host/transform -> host -> roomscanner -> lidar/, so ../../../53L9A1
set(PKG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_library(roomscan_transform SHARED
    rs_transform_shim.c

    # Transform Middleware (algo) -- same source list as
    # firmware/scanner-stream/CMakeLists.txt's PKG_ROOT-prefixed entries
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/confidence.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/depth16.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/distance_calibration.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/distance_check.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/dmax.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/extract.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/flying_pixel.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/radial_to_perp.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/ratenorm.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/reflectance.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/sharpener.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/tnr.c

    # Transform Middleware (main)
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/vl53l9_calib_utils.c
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/vl53l9_transform.c
)

target_include_directories(roomscan_transform PRIVATE
    # Middleware Header Library Includes (matches firmware/scanner-stream/CMakeLists.txt)
    ${PKG_ROOT}/Middlewares/ST/media-object/media-c/header-lib/include/public
    ${PKG_ROOT}/Middlewares/ST/media-object/transform-c/header-lib/include/public
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/private
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/public
)

# Float model: /fp:precise is the MSVC default -- kept explicit here so
# Task 4's /fp:fast experiment is a one-line diff against a documented
# baseline, not a guess about defaults.
target_compile_options(roomscan_transform PRIVATE $<$<C_COMPILER_ID:MSVC>:/fp:precise>)

# Windows DLL export macro is handled in rs_transform_shim.h via __declspec(dllexport);
# no additional *_EXPORTS define required since only that header is dllexport-annotated.
