cmake_minimum_required(VERSION 3.19)
project(hello_pc)

include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/eigen)
include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/happly)
include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/nanoflann/include)

add_executable(hello_pc main.cc)
