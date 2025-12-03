cmake_minimum_required(VERSION 3.10)

project(UAVSphereSim LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable more warnings (optional but helpful)
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find OpenGL and GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Threads (for std::thread)
find_package(Threads REQUIRED)

# Your executable
add_executable(uav_sim
    main.cpp
    simulation.cpp
)

# Include current directory for headers (control.h, simulation.h)
target_include_directories(uav_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link libraries
target_link_libraries(uav_sim PRIVATE
    OpenGL::GL
    GLUT::GLUT
    Threads::Threads
    GLU            # <- add this
)

# On some systems you may also need GLU.
# If linking fails complaining about glu, you can add:
#   find_package(OpenGL REQUIRED GLU)
# and then also link OpenGL::GLU
