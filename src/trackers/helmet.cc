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

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <boost/numeric/conversion/converter.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>


// uBlas extension
#include "storage_adaptors.hpp"

extern "C"
{
#include <evart-client.h>
}

#include "corba-signal.hh"

#define ENABLE_DEBUG
#include "debug.hh"

#include "helmet.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

namespace helmet
{
  boost::mt19937 gen;
} // end of namespace helmet


static const std::string name ("helmet");

TRACKED_BODY_IMPL (Helmet, name);

Helmet::Helmet (Application& app)
  : TrackedBody (app, name, "helmet", 4),
    fr_ (0),
    fl_ (1),
    t_ (2),
    b_ (3)
{}

Helmet::~Helmet ()
{}

void
Helmet::computeSignal (const evas_msg_t* msg)
{
  marker_t* markers = (marker_t*) &msg->body_markers.data
    + msg->body_markers.markersOffset;
  vector_t fr = ublas::make_vector_from_pointer
    (3, markers[0]);
  vector_t fl = ublas::make_vector_from_pointer
    (3, markers[0]);
  vector_t t = ublas::make_vector_from_pointer
    (3, markers[0]);
  vector_t b = ublas::make_vector_from_pointer
    (3, markers[0]);

  vector_t t_ (3);
  t_[0] = b[0];
  t_[1] = t[1];
  t_[2] = t[2];

  signalOutput_->length (6);
  signalOutput_[0] = t[0];
  signalOutput_[1] = t[1];
  signalOutput_[2] = t[2];
  signalOutput_[3] = 0.;
  signalOutput_[4] = 0.;
  signalOutput_[5] = atan2 (ublas::norm_2 (t_- b),
			    ublas::norm_2 (t_- b));

  signalTimestampOutput_->length (2);
  signalTimestampOutput_[0] = msg->body_markers.tv_sec;
  signalTimestampOutput_[1] = msg->body_markers.tv_usec;
}

void
Helmet::simulateSignal ()
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::posix_time::ptime ptime_t;

  static const double mean = 0.;
  static const double standard_deviation = 1.;
  boost::normal_distribution<> dist (mean, standard_deviation);

  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
    die (helmet::gen, dist);

  signalOutput_->length (3);
  for (unsigned i = 0; i < 3; ++i)
    signalOutput_[i] =  die ();

  signalTimestampOutput_->length (2);

  ptime_t time =
    boost::posix_time::microsec_clock::universal_time ();
  int64_t sec = time.time_of_day ().total_seconds();
  int64_t usec = time.time_of_day ().total_microseconds () - sec * 1000000;

  typedef boost::numeric::converter<double, int64_t> Int64_t2Double;

  signalTimestampOutput_[0] = Int64_t2Double::convert (sec);
  signalTimestampOutput_[1] = Int64_t2Double::convert (usec);
}
