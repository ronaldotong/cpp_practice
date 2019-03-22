cmake_minimum_required(VERSION 3.0.0)
project(cppbase VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(cppbase main.cpp graph.hpp sort.hpp)
target_link_libraries(cppbase m)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
