# Build Instructions

This project uses CMake for cross-platform build configuration.

## Prerequisites

- **CMake** (4.2 or higher)
- **C++ Compiler** supporting C++20 (GCC, Clang, MSVC)
- **Dependencies**:
    - **Windows**: Dependencies are included in the `Dependencies` folder.
    - **Linux**: You need to install `libglfw3-dev`, `freeglut3-dev`, `libgl1-mesa-dev`, `libglu1-mesa-dev`, `xorg-dev`.
    - **macOS**: `glfw` and `freeglut` (via Homebrew).

## Building

1. Create a build directory:
   ```bash
   cmake -S . -B build
   ```

2. Compile the project:
   ```bash
   cmake --build build
   ```

## Running

- **Windows**: Run `build\Debug\TrafficSimulator.exe` (or `Release` if configured).
- **Linux/macOS**: Run `./build/TrafficSimulator`

## IDE Support

You can open this folder directly in:
- **Visual Studio**: Open the folder. VS will detect CMake.
- **VS Code**:
    - **Option A (Recommended)**: Open the folder and press **F5** to build and debug. (Requires C/C++ extension).
    - **Option B**: Install "CMake Tools" extension for advanced CMake integration.
- **CLion**: Open `CMakeLists.txt` as a project.
