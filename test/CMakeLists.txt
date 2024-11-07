#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesGettingStarted)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


find_library(REALSENSE2_FOUND realsense2 REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

find_package(OpenGL REQUIRED)

find_package(glfw3 REQUIRED)
# getting-started
add_executable(rs-getting-started exampel.cpp)
message("INCLUDE DIRS: ${REALSENSE2_FOUND}")
# include_directories(rs-getting-started ${ADDITIONAL_INCLUDE_DIRS})
include_directories(.inc/common ${LZ4_DIR})
include_directories(rs-getting-started ./inc)
include_directories(rs-getting-started ./inc/common)
target_link_libraries(rs-getting-started ${REALSENSE2_FOUND})
target_link_libraries(rs-getting-started ${glfw3})
target_link_libraries(rs-getting-started ${OpenGL})
target_link_libraries(rs-getting-started ${CMAKE_DL_LIBS})

# target_link_libraries(rs-getting-started
#   ${BINARY_DIR}/libgmock.a
#   glog
#   gflags
#   ${Boost_LIBRARIES}
#   ${CMAKE_THREAD_LIBS_INIT}
# )
