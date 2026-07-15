cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME scanner_stream)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 53L9A1 reference package (read-only) providing shared Utilities/Middlewares/BSP components
set(PKG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/vl53l9_app.c
    Src/rs_protocol.c
    Src/rs_lsm.c

    # LSM6DSV16X register driver (vendored from X-NUCLEO-IKS4A1, unedited)
    Drivers/lsm6dsv16x/lsm6dsv16x_reg.c

    # Utilities
    ${PKG_ROOT}/Utilities/vl53l9-common/platform/platform_utils.c
    ${PKG_ROOT}/Utilities/vl53l9-common/vl53l9/vl53l9_device.c
    ${PKG_ROOT}/Utilities/vl53l9-common/vl53l9/vl53l9_platform.c
    ${PKG_ROOT}/Utilities/vl53l9-common/vl53l9/vl53l9_utils.c

    # BSP
    ${PKG_ROOT}/Drivers/BSP/Components/vl53l9/vl53l9.c

    # Transform Middleware (algo)
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

    Src/usb_descriptors.c

    # TinyUSB (vendored, MIT)
    ../vendor/tinyusb/src/tusb.c
    ../vendor/tinyusb/src/common/tusb_fifo.c
    ../vendor/tinyusb/src/device/usbd.c
    ../vendor/tinyusb/src/class/cdc/cdc_device.c
    ../vendor/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ../vendor/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
)

# Find LwIP sources
file(GLOB_RECURSE LWIP_SRCS
    "../vendor/lwip/src/core/*.c"
    "../vendor/lwip/src/core/ipv4/*.c"
    "../vendor/lwip/src/netif/ethernet.c"
    "../vendor/lwip/src/apps/mdns/mdns.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${LWIP_SRCS}
    ${PKG_ROOT}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_eth.c
    ${PKG_ROOT}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_eth_ex.c
    Src/ethernetif.c
    Src/ethernetif.c
    Src/ethernet_transport.c
    Src/dhcpserver.c
    Src/lan8742.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Inc
    Src
    Drivers/lsm6dsv16x
    ${PKG_ROOT}/Drivers/BSP/Components/vl53l9
    ${PKG_ROOT}/Utilities/vl53l9-common
    ${PKG_ROOT}/Utilities/vl53l9-common/vl53l9

    # Middleware Header Library Includes
    ${PKG_ROOT}/Middlewares/ST/media-object/media-c/header-lib/include/public
    ${PKG_ROOT}/Middlewares/ST/media-object/transform-c/header-lib/include/public
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/private
    ${PKG_ROOT}/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/public

    # TinyUSB (vendored, MIT)
    ../vendor/tinyusb/src

    # lwIP
    ../vendor/lwip/src/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_NUCLEO_144
    CFG_TUSB_MCU=OPT_MCU_STM32H5
)

# Optimization options -Ofast for real-time performance on ToF data
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Ofast
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# Post-build commands: Generate .bin file and print code size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .bin and printing executable size..."
)
