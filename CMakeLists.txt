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
find_package(qibuild)

include_directories(".")
qi_create_bin(pepper_control main.cpp control.hpp control.cpp)
qi_use_lib(pepper_control QI ALCOMMON ALPROXIES)
