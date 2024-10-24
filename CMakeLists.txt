cmake_minimum_required(VERSION 3.10)
project(collision-detection)

# Create the library target
add_executable(collision_detection src/cps_collision_detection.c)

# Add subdirectories for headers and sources
add_subdirectory(inc)
