cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(backward_induction)

#include_directories(cmake)

#add_executable(backward_induction main.cpp model.cpp)
add_executable(backward_induction forward_search.cpp model.cpp)
set_property(TARGET backward_induction PROPERTY CXX_STANDARD 11)