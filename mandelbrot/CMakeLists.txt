# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(mandelbrot-project)

# initialize the sdk
pico_sdk_init()

#######

# name anything you want
add_executable(mandelbrot)

# must match with pio filename and executable name from above
pico_generate_pio_header(mandelbrot ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(mandelbrot ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(mandelbrot ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
pico_generate_pio_header(mandelbrot ${CMAKE_CURRENT_LIST_DIR}/cgain.pio)
pico_generate_pio_header(mandelbrot ${CMAKE_CURRENT_LIST_DIR}/cgah.pio)

# must match with executable name and source file names
target_sources(mandelbrot PRIVATE mandelbrot.c)

# must match with executable name
target_link_libraries(mandelbrot PRIVATE pico_stdlib hardware_pio hardware_dma)

# must match with executable name
pico_add_extra_outputs(mandelbrot)

pico_set_binary_type(mandelbrot no_flash)
pico_enable_stdio_usb(mandelbrot 1)
pico_enable_stdio_uart(mandelbrot 0)
