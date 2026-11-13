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

## Tools

Both are headless and built with the project:

- `./build/benchmark` compares the traffic light controllers on the same random networks and
  the same traffic demand (options at the top of `tools/benchmark.cpp`).
- `./build/train dqn` and `./build/train qlearning` train the learning controllers and write
  their models in `models/` (options at the top of `tools/train.cpp`).

Use a Release build (`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`) for these: training the
DQN takes about 3 minutes in Release. Run them from the repository folder, where `models/` is.

## IDE Support

You can open this folder directly in:
- **Visual Studio**: Open the folder. VS will detect CMake.
- **VS Code**:
    - **Option A (Recommended)**: Open the folder and press **F5** to build and debug. (Requires C/C++ extension).
    - **Option B**: Install "CMake Tools" extension for advanced CMake integration.
- **CLion**: Open `CMakeLists.txt` as a project.
