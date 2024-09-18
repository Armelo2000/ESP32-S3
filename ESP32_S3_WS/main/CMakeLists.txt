# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

#idf_component_register(
#    SRCS main.c         # list the source files of this component
#    INCLUDE_DIRS        # optional, add here public include directories
#    PRIV_INCLUDE_DIRS   # optional, add here private include directories
#    REQUIRES            # optional, list the public requirements (component names)
#    PRIV_REQUIRES       # optional, list the private requirements
#)

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)
idf_component_register(SRCS ${app_sources}
						INCLUDE_DIRS "."  "../components/ESP32_Display_Panel" "../components/ESP32_IO_Expander" "../components/lvgl")