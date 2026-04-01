cmake_minimum_required(VERSION 3.13)
project(TrafficFlow)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force dynamic runtime for MSVC
if(MSVC)
    foreach(flag_var
            CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
        string(REPLACE "/MTd" "/MDd" ${flag_var} "${${flag_var}}")
        string(REPLACE "/MT"  "/MD"  ${flag_var} "${${flag_var}}")
    endforeach()
endif()

# Source files
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "headers/*.h")

# Exclude Main.cpp from library sources
set(LIB_SOURCES ${SOURCES})
list(FILTER LIB_SOURCES EXCLUDE REGEX "Main.cpp$")

# ImGui Sources
set(IMGUI_SOURCES
    Dependencies/imgui/imgui.cpp
    Dependencies/imgui/imgui_draw.cpp
    Dependencies/imgui/imgui_tables.cpp
    Dependencies/imgui/imgui_widgets.cpp
    Dependencies/imgui/backends/imgui_impl_glfw.cpp
    Dependencies/imgui/backends/imgui_impl_opengl3.cpp
)

# Create a static library from non-main sources
add_library(TrafficLib STATIC ${LIB_SOURCES} ${HEADERS} ${IMGUI_SOURCES})

# Include directories
target_include_directories(TrafficLib PRIVATE
    headers
    Dependencies/stb/include
    Dependencies/delaunator/include
    Dependencies/imgui
    Dependencies/imgui/backends
)

# Main executable
add_executable(TrafficFlow src/Main.cpp)
target_include_directories(TrafficFlow PRIVATE headers)
target_link_libraries(TrafficFlow PRIVATE TrafficLib)

# Tests
enable_testing()
option(ENABLE_COVERAGE "Enable code coverage" OFF)

if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_library(coverage_flags INTERFACE)
    target_compile_options(coverage_flags INTERFACE
        -O0
        -g
        --coverage
    )
    target_link_options(coverage_flags INTERFACE
        --coverage
    )
endif()

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

file(GLOB TEST_SOURCES "Tests/*.cpp")

add_executable(unit_tests ${TEST_SOURCES})

target_link_libraries(unit_tests
    PRIVATE
    TrafficLib  # Link the project library to tests
    gtest
    gtest_main
)

if(ENABLE_COVERAGE AND TARGET coverage_flags)
    target_link_libraries(unit_tests PRIVATE coverage_flags)
    target_link_libraries(TrafficLib PRIVATE coverage_flags)
    target_link_libraries(TrafficFlow PRIVATE coverage_flags)
endif()

target_include_directories(unit_tests PRIVATE headers)

add_test(NAME unit_tests COMMAND unit_tests)

# Platform specific settings
if(WIN32)
    # Windows Dependencies (MSVC prebuilt)
    # GLFW
    set(GLFW_LIB_DIR "${CMAKE_SOURCE_DIR}/Dependencies/GLFW/lib-vc2022")
    # FreeGLUT
    set(GL_LIB_DIR "${CMAKE_SOURCE_DIR}/Dependencies/GL/lib")
    target_link_directories(TrafficFlow PRIVATE ${GLFW_LIB_DIR} ${GL_LIB_DIR})
    target_link_directories(TrafficLib PRIVATE ${GLFW_LIB_DIR} ${GL_LIB_DIR})  # Also for lib if needed
    target_link_directories(unit_tests PRIVATE ${GLFW_LIB_DIR} ${GL_LIB_DIR})  # Also for tests if needed
    
    target_link_libraries(TrafficFlow PRIVATE
        glfw3
        freeglut
        opengl32
        glu32
        user32
        gdi32
        shell32
    )
    target_link_libraries(TrafficLib PRIVATE
        glfw3
        freeglut
        opengl32
        glu32
        user32
        gdi32
        shell32
    )  # If your lib needs these; adjust as necessary
    # Copy DLLs to output directory
    add_custom_command(TARGET TrafficFlow POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/Dependencies/GL/lib/freeglut.dll"
        $<TARGET_FILE_DIR:TrafficFlow>
    )
    
    # Check if glfw3.dll exists and copy it if so (it might be static)
    if(EXISTS "${GLFW_LIB_DIR}/glfw3.dll")
        add_custom_command(TARGET TrafficFlow POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLFW_LIB_DIR}/glfw3.dll"
            $<TARGET_FILE_DIR:TrafficFlow>
        )
    endif()

    # Include directories
    target_include_directories(TrafficLib PRIVATE
        Dependencies/GLFW/include
        Dependencies/GL/include
    )

    # Warning flags for MSVC
    if(MSVC)
        target_compile_options(TrafficLib PRIVATE /W4 /permissive-)
        target_compile_options(TrafficFlow PRIVATE /W4 /permissive-)
        target_compile_options(unit_tests PRIVATE /W4 /permissive-)
    endif()

elseif(UNIX AND NOT APPLE)
    # Linux Dependencies
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW3 REQUIRED glfw3)

    target_link_libraries(TrafficFlow PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLFW3_LIBRARIES}
        pthread
        X11
        Xrandr
        Xi
        dl
    )
    target_link_libraries(TrafficLib PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLFW3_LIBRARIES}
        pthread
        X11
        Xrandr
        Xi
        dl
    )
    
    # Include directories
    target_include_directories(TrafficFlow PRIVATE ${GLFW3_INCLUDE_DIRS})
    target_include_directories(TrafficLib PRIVATE ${GLFW3_INCLUDE_DIRS})

    # Warning flags for GCC/Clang
    target_compile_options(TrafficLib PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(TrafficFlow PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)

elseif(APPLE)
    # macOS Dependencies (Basic setup, might need adjustment)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(glfw3 REQUIRED)
    
    target_link_libraries(TrafficFlow PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        glfw
    )
    target_link_libraries(TrafficLib PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        glfw
    )

    # Warning flags for Clang
    target_compile_options(TrafficLib PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(TrafficFlow PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)

endif()
