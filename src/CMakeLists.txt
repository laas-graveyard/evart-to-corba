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

include(../cmake/idl.cmake)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")
INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")

GENERATE_IDL_FILE(corba-signal
  "${DYNAMIC_GRAPH_CORBA_DATAROOTDIR}/idl/dynamic-graph")

ADD_EXECUTABLE(evart-to-corba
  corba-connection.cc corba-connection.hh
  application.cc application.hh
  tracked-body.cc tracked-body.hh
  tracked-body-factory.cc tracked-body-factory.hh

  tracked-segment.cc tracked-segment.hh
  tracked-segment-factory.cc tracked-segment-factory.hh

  trackers/waist-tracker.cc trackers/waist-tracker.hh
  trackers/three-points-tracker.cc trackers/three-points-tracker.hh

  trackers/helmet.cc trackers/helmet.hh

  trackers/table-tracker.cc trackers/table-tracker.hh

  segment-trackers/segment-waist-tracker.cc
  segment-trackers/segment-waist-tracker.hh

  segment-trackers/left-foot-tracker.cc
  segment-trackers/left-foot-tracker.hh

  segment-trackers/head-tracker.cc
  segment-trackers/head-tracker.hh

  evart-to-corba.cc
  ${ALL_IDL_STUBS}
  )

PKG_CONFIG_USE_DEPENDENCY(evart-to-corba jrl-mathtools)
PKG_CONFIG_USE_DEPENDENCY(evart-to-corba evart-client)
PKG_CONFIG_USE_DEPENDENCY(evart-to-corba dynamic-graph-corba)
PKG_CONFIG_USE_DEPENDENCY(evart-to-corba omniORB4)

TARGET_LINK_LIBRARIES(evart-to-corba ${Boost_LIBRARIES})

INSTALL(TARGETS evart-to-corba DESTINATION bin)
