cmake_minimum_required(VERSION 3.11.4)
project(dynamic-linking LANGUAGES C CXX)

include(FetchContent)

FetchContent_Declare(WAVM
    GIT_REPOSITORY https://github.com/WAVM/WAVM.git
    GIT_TAG 12c06f113e99a11af3b39fc8181056c1a55c27b6
)

FetchContent_GetProperties(WAVM)
if(NOT wavm_POPULATED)
    FetchContent_Populate(WAVM)
    add_subdirectory(
        ${wavm_SOURCE_DIR}
        ${wavm_BINARY_DIR}
    )
endif()

add_executable(dynamic-linking dynamic-linking.cpp)
target_link_libraries(dynamic-linking libWAVM)

