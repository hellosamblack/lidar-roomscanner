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
set(CMAKE_PROJECT_NAME 53L9A1_PostprocessSingle)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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

    # Utilities
    ../../../../../Utilities/vl53l9-common/platform/platform_utils.c
    ../../../../../Utilities/vl53l9-common/vl53l9/vl53l9_device.c
    ../../../../../Utilities/vl53l9-common/vl53l9/vl53l9_platform.c
    ../../../../../Utilities/vl53l9-common/vl53l9/vl53l9_utils.c

    # BSP
    ../../../../../Drivers/BSP/Components/vl53l9/vl53l9.c

    # Transform Middleware (algo)
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/confidence.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/depth16.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/distance_calibration.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/distance_check.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/dmax.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/extract.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/flying_pixel.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/radial_to_perp.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/ratenorm.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/reflectance.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/sharpener.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/tnr.c

    # Transform Middleware (main)
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/vl53l9_calib_utils.c
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/vl53l9_transform.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Inc
    ../../../../../Drivers/BSP/Components/vl53l9
    ../../../../../Utilities/vl53l9-common
    ../../../../../Utilities/vl53l9-common/vl53l9

    # Middleware Header Library Includes
    ../../../../../Middlewares/ST/media-object/media-c/header-lib/include/public
    ../../../../../Middlewares/ST/media-object/transform-c/header-lib/include/public
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/private
    ../../../../../Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/include/public
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_NUCLEO_144
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
