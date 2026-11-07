# Build Instructions

This project uses CMake for cross-platform build configuration.

## Prerequisites

- **CMake** (3.16 or higher)
- **C++ Compiler** supporting C++20 (GCC, Clang, MSVC, MinGW-w64)
- **Dependencies** (OpenGL + GLFW; GoogleTest is downloaded automatically):
    - **Windows (MSVC)**: GLFW is included in the `Dependencies` folder.
    - **Windows (MinGW / Clang)**: GLFW is downloaded and built automatically (or uses an installed one).
    - **Linux**: `sudo apt install libglfw3-dev libgl1-mesa-dev`
    - **macOS**: `brew install glfw` (configure with `-DCMAKE_PREFIX_PATH=$(brew --prefix)` if not found).

## Building

1. Create a build directory:
   ```bash
   cmake -S . -B build
   ```

2. Compile the project:
   ```bash
   cmake --build build
   ```

3. Run the tests:
   ```bash
   ctest --test-dir build --output-on-failure -C Debug
   ```

## Running

- **Windows**: Run `build\Debug\TrafficFlow.exe` (or `Release` if configured).
- **Linux/macOS**: Run `./build/TrafficFlow`

## IDE Support

You can open this folder directly in:
- **Visual Studio**: Open the folder. VS will detect CMake.
- **VS Code**:
    - **Option A (Recommended)**: Open the folder and press **F5** to build and debug. (Requires C/C++ extension).
    - **Option B**: Install "CMake Tools" extension for advanced CMake integration.
- **CLion**: Open `CMakeLists.txt` as a project.
