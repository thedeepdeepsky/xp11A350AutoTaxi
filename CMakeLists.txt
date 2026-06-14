cmake_minimum_required(VERSION 3.18)
project(A350AutoTaxi VERSION 0.2.0 LANGUAGES CXX)

# Usage in CLion:
#   File -> Open -> this folder
#   CMake options:
#     -DXPLANE_SDK_DIR="D:/XPlaneSDK"
#
# The SDK folder must contain:
#   CHeaders/XPLM
#   CHeaders/Wrappers
#   Libraries/Win/XPLM_64.lib   (Windows)
#   Libraries/Mac/XPLM.framework (macOS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED XPLANE_SDK_DIR)
    message(FATAL_ERROR "Please configure -DXPLANE_SDK_DIR=/path/to/XPlaneSDK")
endif()

add_library(A350AutoTaxi MODULE
        src/Plugin.cpp
        src/AutoTaxiUI.cpp
        src/AutoTaxi.cpp
        src/AptDatParser.cpp
        src/Config.cpp
        src/Geo.cpp
)

target_include_directories(A350AutoTaxi PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${XPLANE_SDK_DIR}/CHeaders/XPLM
        ${XPLANE_SDK_DIR}/CHeaders/Wrappers
)

# Target X-Plane 11.50+ with SDK 3.03 APIs.
target_compile_definitions(A350AutoTaxi PRIVATE
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM303=1
)

if(WIN32)
    target_compile_definitions(A350AutoTaxi PRIVATE IBM=1)
    target_link_libraries(A350AutoTaxi PRIVATE "${XPLANE_SDK_DIR}/Libraries/Win/XPLM_64.lib" opengl32)

    # MinGW builds otherwise often depend on libstdc++-6.dll, libgcc_s_seh-1.dll
    # and libwinpthread-1.dll. X-Plane may then fail to LoadLibrary() the plugin
    # with Error Code 126/127 if a runtime DLL is missing or the wrong version is
    # found first in PATH. Static GCC runtimes make the produced win.xpl portable.
    if(MINGW)
        target_link_options(A350AutoTaxi PRIVATE -static -static-libgcc -static-libstdc++ -Wl,--kill-at)
    endif()

    set(XP_PLUGIN_BINARY_NAME "win")
elseif(APPLE)
    target_compile_definitions(A350AutoTaxi PRIVATE APL=1)
    target_link_options(A350AutoTaxi PRIVATE "-F${XPLANE_SDK_DIR}/Libraries/Mac")
    target_link_libraries(A350AutoTaxi PRIVATE "-framework XPLM" "-framework OpenGL")
    set(XP_PLUGIN_BINARY_NAME "mac")
else()
    target_compile_definitions(A350AutoTaxi PRIVATE LIN=1)
    # Linux XPLM symbols are resolved by X-Plane at runtime; OpenGL is used only in the SDK window draw callback.
    target_link_libraries(A350AutoTaxi PRIVATE GL)
    set(XP_PLUGIN_BINARY_NAME "lin")
endif()

set_target_properties(A350AutoTaxi PROPERTIES
        PREFIX ""
        SUFFIX ".xpl"
        OUTPUT_NAME "${XP_PLUGIN_BINARY_NAME}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/A350AutoTaxi/64"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/A350AutoTaxi/64"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/A350AutoTaxi/lib"
)

add_custom_command(TARGET A350AutoTaxi POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/plugin/A350AutoTaxi.ini"
        "${CMAKE_BINARY_DIR}/A350AutoTaxi/A350AutoTaxi.ini"
        COMMAND ${CMAKE_COMMAND} -E echo
        "Built plugin folder: ${CMAKE_BINARY_DIR}/A350AutoTaxi"
)
