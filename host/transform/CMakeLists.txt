cmake_minimum_required(VERSION 3.22)
project(roomscan_transform C)

# Vendor package (read-only) providing the vl53l9-transform-c / media-object
# sources. Compiled in place; never edited.
#
# Two packages in the tree carry the same library at different versions:
#   53L9A1       -> vl53l9-transform-c 1.3.1 (the NUCLEO-H563ZI reference our
#                   firmware is forked from; what shipped up to 2026-08-10)
#   STSW-IMG053  -> vl53l9-transform-c 1.5.0 (the STM32N6570-DK package)
# Both expose the same public API and the same media-object include layout, so
# the shim builds unmodified against either. Select with
#   cmake -S host/transform -B <dir> -DRS_TRANSFORM_PKG=STSW-IMG053
# host/transform -> host -> roomscanner -> firmware/vendor/<pkg>
set(RS_TRANSFORM_PKG "STSW-IMG053" CACHE STRING
    "vl53l9-transform-c source package: 53L9A1 (1.3.1) or STSW-IMG053 (1.5.0)")
set_property(CACHE RS_TRANSFORM_PKG PROPERTY STRINGS 53L9A1 STSW-IMG053)

set(VENDOR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vendor)
if(RS_TRANSFORM_PKG STREQUAL "53L9A1")
    set(MEDIA_ROOT ${VENDOR_ROOT}/53L9A1/Middlewares/ST/media-object)
    set(XFORM_ROOT ${VENDOR_ROOT}/53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib)
elseif(RS_TRANSFORM_PKG STREQUAL "STSW-IMG053")
    set(N6_ROOT ${VENDOR_ROOT}/stsw-img053/vl53l9-stm32n6570-dk-1.0.1)
    set(MEDIA_ROOT ${N6_ROOT}/lib/media-object)
    set(XFORM_ROOT ${N6_ROOT}/lib/transform/vl53l9-transform-c/vl53l9-transform-c-lib)
else()
    message(FATAL_ERROR "RS_TRANSFORM_PKG must be 53L9A1 or STSW-IMG053, got '${RS_TRANSFORM_PKG}'")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_library(roomscan_transform SHARED
    rs_transform_shim.c

    # Transform Middleware (algo) -- same source list as
    # firmware/scanner-stream/CMakeLists.txt's PKG_ROOT-prefixed entries
    ${XFORM_ROOT}/src/algo/confidence.c
    ${XFORM_ROOT}/src/algo/depth16.c
    ${XFORM_ROOT}/src/algo/distance_calibration.c
    ${XFORM_ROOT}/src/algo/distance_check.c
    ${XFORM_ROOT}/src/algo/dmax.c
    ${XFORM_ROOT}/src/algo/extract.c
    ${XFORM_ROOT}/src/algo/flying_pixel.c
    ${XFORM_ROOT}/src/algo/radial_to_perp.c
    ${XFORM_ROOT}/src/algo/ratenorm.c
    ${XFORM_ROOT}/src/algo/reflectance.c
    ${XFORM_ROOT}/src/algo/sharpener.c
    ${XFORM_ROOT}/src/algo/tnr.c

    # Transform Middleware (main)
    ${XFORM_ROOT}/src/vl53l9_calib_utils.c
    ${XFORM_ROOT}/src/vl53l9_transform.c
)

target_include_directories(roomscan_transform PRIVATE
    # Middleware Header Library Includes (matches firmware/scanner-stream/CMakeLists.txt)
    ${MEDIA_ROOT}/media-c/header-lib/include/public
    ${MEDIA_ROOT}/transform-c/header-lib/include/public
    ${XFORM_ROOT}/include/private
    ${XFORM_ROOT}/include/public
)

# vl53l9_transform.h self-defines VL53L9_TRANSFORM_LIGHT to 1 when the caller
# leaves it undefined, so every build of this shim up to 2026-08-10 was a LIGHT
# build without ever saying so. In 1.3.1 that only blanked the control nick /
# description strings. In 1.5.0 it ALSO flips the DEFAULTS of bypass-tnr-algo
# and bypass-flying-pixel-filter to true -- silently disabling the temporal
# denoiser and the flying-pixel filter. That is not a knob we want: TNR-off
# measures 2.9x the depth temporal noise and worse SLAM closure (0.65 -> 0.83 m
# on coffeeRoomCircuitNoMnt, n=5). Pinned off, which is what made 1.5.0
# bit-identical to the 1.3.1 output we shipped -- 1800 frames across three
# captures and both ranging modes, all four output planes, zero difference.
# Setting it explicitly is also a no-op for 1.3.1 (verified bit-identical), so
# both RS_TRANSFORM_PKG values stay directly comparable.
target_compile_definitions(roomscan_transform PRIVATE VL53L9_TRANSFORM_LIGHT=0)

# Float model: /fp:precise is the MSVC default -- kept explicit here so
# Task 4's /fp:fast experiment is a one-line diff against a documented
# baseline, not a guess about defaults.
target_compile_options(roomscan_transform PRIVATE $<$<C_COMPILER_ID:MSVC>:/fp:precise>)

# Windows DLL export macro is handled in rs_transform_shim.h via __declspec(dllexport);
# no additional *_EXPORTS define required since only that header is dllexport-annotated.
