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

#include "segment-torus1-tracker.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

namespace torus1TrackerSegment
{
  boost::mt19937 gen;
} // end of namespace torus1TrackerSegment.

TRACKED_SEGMENT_IMPL (Torus1TrackerSegment, "torus1", "PO");

Torus1TrackerSegment::Torus1TrackerSegment (Application& app)
  : TrackedSegment (app,
		    Torus1TrackerSegment::BODY_NAME,
		    Torus1TrackerSegment::BODY_NAME,
		    Torus1TrackerSegment::SEGMENT_NAME,
                    4, 1)
{}

Torus1TrackerSegment::~Torus1TrackerSegment ()
{}


void
Torus1TrackerSegment::computeSignal (const evas_msg_t* msg)
{
  vector_t data = ublas::make_vector_from_pointer
    (7, msg->body_segments.segments[segmentId ()].pos);

  for (unsigned i = 0; i < 4; ++i)
    if (data[i] == EVAS_EMPTY)
      return;

#ifdef EVART_TO_CORBA_FULL_CONFIGURATION
  signalOutput_->length (7);
  for (unsigned i = 0; i < 7; ++i)
    {
      signalOutput_[i] = data[i];

      // Convert to SI units.
      if (i < 3 || i == 6)
	signalOutput_[i] /= 1000.;
      else
	signalOutput_[i] *= M_PI / 180.;
    }
#else
  signalOutput_->length (3);
  for (unsigned i = 0; i < 2; ++i)
    {
      signalOutput_[i] = data[i];
      signalOutput_[i] /= 1000.;
    }

  signalOutput_[2] = data[5];
  signalOutput_[2] *= M_PI / 180.;
#endif // EVART_TO_CORBA_FULL_CONFIGURATION

  signalTimestampOutput_->length (2);
  signalTimestampOutput_[0] = msg->body_segments.tv_sec;
  signalTimestampOutput_[1] = msg->body_segments.tv_usec;
}

void
Torus1TrackerSegment::simulateSignal ()
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::posix_time::ptime ptime_t;

  static const double mean = 0.;
  static const double standard_deviation = 1.;
  boost::normal_distribution<> dist (mean, standard_deviation);

  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
    die (torus1TrackerSegment::gen, dist);

#ifdef EVART_TO_CORBA_FULL_CONFIGURATION
  signalOutput_->length (7);
  for (unsigned i = 0; i < 7; ++i)
    signalOutput_[i] =  die ();
#else
  signalOutput_->length (3);
  for (unsigned i = 0; i < 3; ++i)
    signalOutput_[i] =  die ();
#endif // EVART_TO_CORBA_FULL_CONFIGURATION

  signalTimestampOutput_->length (2);

  ptime_t time =
    boost::posix_time::microsec_clock::universal_time ();
  std::vector<int64_t> time_ = to_timeval (time);
  typedef boost::numeric::converter<double, int64_t> Int64_t2Double;
  signalTimestampOutput_[0] = Int64_t2Double::convert (time_[0]);
  signalTimestampOutput_[1] = Int64_t2Double::convert (time_[1]);
}
