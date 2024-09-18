# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#list(APPEND EXTRA_COMPONENT_DIRS components/ESP32_Display_Panel components/ESP32_IO_Expander components/lvgl)
list(APPEND EXTRA_COMPONENT_DIRS components/ESP32_Display_Panel components/ESP32_IO_Expander components/lvgl)
project(ESP32_S3_WS_app)

# Define a global macro for the project
add_compile_definitions(ESP_PANEL_USE_BOARD)