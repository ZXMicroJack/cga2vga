cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
# include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_8BA -DSAMPLE_RATE=22050 -DSAMPLE_LEN=512 -DPICO_AUDIO_I2S_DATA_PIN=4 -DPICO_AUDIO_I2S_CLOCK_PIN_BASE=2 -DVE_STEPS=64 -DLFO_STEPS=64 -DNONAME -DPREFETCH")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_SCANVIDEO_COLOR_PIN_BASE=0 -DPICO_SCANVIDEO_COLOR_PIN_COUNT=16 ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_SCANVIDEO_PIXEL_RCOUNT=5 -DPICO_SCANVIDEO_PIXEL_RSHIFT=0 ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_SCANVIDEO_PIXEL_GCOUNT=5 -DPICO_SCANVIDEO_PIXEL_GSHIFT=6 ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_SCANVIDEO_PIXEL_BCOUNT=5 -DPICO_SCANVIDEO_PIXEL_BSHIFT=11 ")

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

include(example_auto_set_url.cmake)

# Build project

#include_directories(. ../engine ../sf2 .. ../pm_embedded)

# midi module
add_executable(scanvideo_minimal scanvideo_minimal.c)
target_link_libraries(scanvideo_minimal PRIVATE pico_multicore pico_stdlib pico_scanvideo_dpi)
add_compile_options(-O3)
pico_enable_stdio_usb(scanvideo_minimal 1)
pico_enable_stdio_uart(scanvideo_minimal 0)
pico_add_extra_outputs(scanvideo_minimal)
example_auto_set_url(scanvideo_minimal)
target_compile_options(scanvideo_minimal PUBLIC "-DZXUNO -DRAM_BUILD")
pico_set_binary_type(scanvideo_minimal no_flash)


