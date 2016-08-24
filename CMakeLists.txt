# In CMakeLists.txt

cmake_minimum_required(VERSION 2.8)
project(pepper_control)
find_package(qibuild)

include_directories(".")
qi_create_bin(pepper_control main.cpp control.hpp control.cpp)
qi_use_lib(pepper_control QI ALCOMMON ALPROXIES)
