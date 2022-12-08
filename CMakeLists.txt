cmake_minimum_required(VERSION 3.16)

project(test VERSION 1.0)

set(CMAKE_CXX_FLAGS "-std=c++2a")
set(CMAKE_CXX_FLAGS -pthread)

find_package (glog 0.6.0 REQUIRED)

include_directories(./inc/)

include_directories(${GLOG_INCLUDE_DIRS})

link_directories(./)

set(TSRC ./test.cpp
        )

# add_subdirectory(gflags)

add_executable(test ${TSRC})
target_link_libraries(test glog::glog pthread)
target_link_libraries(test controlcan motion_ctrl)

# target_link_libraries(motion_ctrl gflags::gflags)
target_link_libraries(test boost_system boost_thread boost_chrono pthread)
