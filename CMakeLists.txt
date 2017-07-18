#############################################################################
#
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using this software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, or concerning the
# license please contact Inria at Fabien.Spindler@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Authors:
# Fabien Spindler, Giovanni Claudio
#
#############################################################################

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
)

add_executable(pepper_control main.cpp control.hpp  control.cpp )

target_link_libraries(pepper_control
  ${naoqi_libqi_LIBRARIES}
  ${naoqi_libqicore_LIBRARIES}
  ${Boost_LIBRARIES}
)
