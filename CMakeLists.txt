# In CMakeLists.txt

cmake_minimum_required(VERSION 2.8)
project(pepper_control)

find_package(naoqi_libqi)

find_package(naoqi_libqicore)

find_package(Boost REQUIRED filesystem)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

include_directories(
  ${naoqi_libqi_INCLUDE_DIRS}
  ${naoqi_libqicore_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  "."
)

add_executable(pepper_control main.cpp control.hpp  control.cpp )

target_link_libraries(pepper_control
  ${naoqi_libqi_LIBRARIES}
  ${naoqi_libqicore_LIBRARIES}
  ${Boost_LIBRARIES}
)
