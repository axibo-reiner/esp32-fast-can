cmake_minimum_required(VERSION 3.10)
project(template_project)

# Enable debug symbols
# set(CMAKE_BUILD_TYPE Debug)

# # Enable verbose output
# set(CMAKE_VERBOSE_MAKEFILE ON)

# # Enable additional debug information
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

# # Enable all warnings
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directory
include_directories(include)

# Add source files
set(SOURCES
    src/main.c
)

# Create the executable
add_executable(template_project ${SOURCES} include)

# Link the math library
target_link_libraries(template_project m)
