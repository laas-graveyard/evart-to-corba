// Copyright 2011, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
//
// This file is part of evart-to-corba.
// evart-to-corba is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// evart-to-corba is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// evart-to-corba. If not, see <http://www.gnu.org/licenses/>.

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <boost/numeric/ublas/io.hpp>

// uBlas extension
#include "storage_adaptors.hpp"

#include <evart-client.h>

#include "corba-signal.hh"

#define ENABLE_DEBUG
#include "debug.hh"

#include "table-tracker.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

static const double TABLE_WIDTH = 0.3;
static const double TABLE_LENGTH = 0.3;
static const double TABLE_HEIGHT = 0.25;

TableTracker::TableTracker (Application& app)
  : TrackedBody (app, "tablePosition", 1, 3),
    front_ (0),
    leftUp_ (1),
    rightUp_ (2)
{}

TableTracker::~TableTracker ()
{}

void
TableTracker::computeSignal (const evas_msg_t* msg)
{
/**
    2 o----o  1       x <---------------o
      |    |                            |
      |    |                           \|/
      |    |                            y
      `----o  0
**/

  vector_t point0 = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[0]);
  vector_t point1 = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[1]);
  vector_t point2 = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[2]);



  double originX = (point0[0] + point2[0])/2.0;
  double originY = (point0[1] + point2[1])/2.0;
  double theta;
  if( fabs(point1[1] - point2[1]) != 0.0 ) theta = atan( (point1[0] - point2[0]) / (point1[1] - point2[1]));
  else theta = M_PI / 2.0;

  signalOutput_->length (3);
  signalOutput_[0] = originX;
  signalOutput_[1] = originY;
  signalOutput_[2] = theta;

  LOG ()
    << "-> "
    << originX << " | "
    << originY << " | "
    << theta * 180. / M_PI << " deg" << std::endl;
}
