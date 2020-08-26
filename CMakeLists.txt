cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(frozenlake)

add_executable(frozenlake hello.cpp)
set_property(TARGET frozenlake PROPERTY CXX_STANDARD 11)