cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(backward_induction VERSION 0.1.0)

#include_directories(cmake)
include(CTest)
enable_testing()

set(XML_FILE "./tinyxml2/tinyxml2.cpp" "./tinyxml2/tinyxml2.h")

#add_executable(backward_induction dp_solver.cpp phy_model.cpp ${XML_FILE})
add_executable(backward_induction main.cpp dp_solver.cpp forward_search.cpp phy_model.cpp ${XML_FILE})
set_property(TARGET backward_induction PROPERTY CXX_STANDARD 11)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
# Solve the "program cannot find an entry for __cxa_throw_bad_array_new_length ... error"
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -v -static-libstdc++")