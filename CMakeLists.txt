cmake_minimum_required(VERSION 3.16)
project(TrafficFlow LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_COVERAGE "Enable code coverage (GCC/Clang only)" OFF)

# Use the dynamic MSVC runtime everywhere (must match GoogleTest and the prebuilt GLFW).
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# ----------------------------------------------------------------------------
# Third-party dependencies: OpenGL + GLFW
# ----------------------------------------------------------------------------
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

if(WIN32 AND MSVC AND EXISTS "${CMAKE_SOURCE_DIR}/Dependencies/GLFW/lib-vc2022/glfw3.lib")
    # Prebuilt static GLFW for MSVC (x64) shipped in the repository.
    add_library(glfw STATIC IMPORTED)
    set_target_properties(glfw PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/Dependencies/GLFW/lib-vc2022/glfw3.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/Dependencies/GLFW/include"
        INTERFACE_LINK_LIBRARIES "user32;gdi32;shell32")
elseif(APPLE)
    find_package(glfw3 3.3 REQUIRED) # brew install glfw
elseif(UNIX)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW3 REQUIRED IMPORTED_TARGET glfw3) # apt install libglfw3-dev
    add_library(glfw INTERFACE IMPORTED)
    set_target_properties(glfw PROPERTIES INTERFACE_LINK_LIBRARIES PkgConfig::GLFW3)
else()
    # Windows with MinGW / Clang: the prebuilt glfw3.lib is MSVC-only, so use an
    # installed GLFW if any, otherwise build it from source.
    find_package(glfw3 3.3 CONFIG QUIET)
    if(NOT TARGET glfw)
        include(FetchContent)
        FetchContent_Declare(
            glfw
            URL https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(glfw)
    endif()
endif()

# ----------------------------------------------------------------------------
# Common compile settings
# ----------------------------------------------------------------------------
add_library(project_options INTERFACE)
target_compile_definitions(project_options INTERFACE
    $<$<PLATFORM_ID:Windows>:NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS>
    $<$<PLATFORM_ID:Darwin>:GL_SILENCE_DEPRECATION>)

add_library(project_warnings INTERFACE)
if(MSVC)
    target_compile_options(project_warnings INTERFACE /W4 /permissive- /utf-8)
else()
    target_compile_options(project_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(project_options INTERFACE -O0 -g --coverage)
    target_link_options(project_options INTERFACE --coverage)
endif()

# ----------------------------------------------------------------------------
# ImGui (third-party: no project warnings)
# ----------------------------------------------------------------------------
add_library(imgui STATIC
    Dependencies/imgui/imgui.cpp
    Dependencies/imgui/imgui_draw.cpp
    Dependencies/imgui/imgui_tables.cpp
    Dependencies/imgui/imgui_widgets.cpp
    Dependencies/imgui/backends/imgui_impl_glfw.cpp
    # The simulator draws with the fixed-function pipeline, so use the matching
    # OpenGL2 renderer: it works with any GL context (incl. Windows' GL 1.1 and
    # macOS legacy contexts) and needs no function loader.
    Dependencies/imgui/backends/imgui_impl_opengl2.cpp)
target_include_directories(imgui PUBLIC Dependencies/imgui Dependencies/imgui/backends)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL PRIVATE project_options)

# ----------------------------------------------------------------------------
# Simulator library (everything except main)
# ----------------------------------------------------------------------------
file(GLOB LIB_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(FILTER LIB_SOURCES EXCLUDE REGEX "Main\\.cpp$")
file(GLOB HEADERS CONFIGURE_DEPENDS "headers/*.h")

add_library(TrafficLib STATIC ${LIB_SOURCES} ${HEADERS})
target_include_directories(TrafficLib
    PUBLIC headers
    PRIVATE Dependencies/stb/include Dependencies/delaunator/include)
target_link_libraries(TrafficLib
    PUBLIC project_options imgui glfw OpenGL::GL
    PRIVATE project_warnings)

# ----------------------------------------------------------------------------
# Executable
# ----------------------------------------------------------------------------
add_executable(TrafficFlow src/Main.cpp)
target_link_libraries(TrafficFlow PRIVATE TrafficLib project_warnings)

# Run from the source tree by default so Graphics/ is found.
set_target_properties(TrafficFlow PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Headless benchmark comparing the traffic light controllers (not run by the tests)
add_executable(benchmark tools/benchmark.cpp)
target_link_libraries(benchmark PRIVATE TrafficLib project_warnings)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
enable_testing()

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "Tests/*.cpp")
add_executable(unit_tests ${TEST_SOURCES})
target_link_libraries(unit_tests PRIVATE TrafficLib project_warnings GTest::gtest_main)

add_test(NAME unit_tests COMMAND unit_tests WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
set_tests_properties(unit_tests PROPERTIES TIMEOUT 600)
