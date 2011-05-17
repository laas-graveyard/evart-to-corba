# Copyright 2011, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of evart-to-corba.
# evart-to-corba is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# evart-to-corba is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# evart-to-corba. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/pthread.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME evart-to-corba)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "http://github.com/laas/evart-to-corba")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-corba")
ADD_REQUIRED_DEPENDENCY("evart-client >= 2.0")
ADD_REQUIRED_DEPENDENCY("omniORB4")

# Search for Boost.
SET(BOOST_COMPONENTS
  filesystem system thread program_options unit_test_framework)
SEARCH_FOR_BOOST()

PKG_CONFIG_APPEND_LIBS("evart-to-corba")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
