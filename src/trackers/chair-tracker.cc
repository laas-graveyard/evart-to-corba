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

#include "chair-tracker.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

namespace chair
{
  boost::mt19937 gen;
} // end of namespace chair

TRACKED_BODY_IMPL (ChairTracker, "chair");

ChairTracker::ChairTracker (Application& app)
  : TrackedBody (app, "chairPosition", ChairTracker::BODY_NAME, 5)
{}

ChairTracker::~ChairTracker ()
{}

void
ChairTracker::computeSignal (const evas_msg_t* msg)
{
/**
   BR o---------o TR       x <-------------o
      |      |  |                          |
      |  BM o|  |                         \|/
      |      |  |                          y
   BL o---------o TL
**/

  vector_t topL     = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[0]);
  vector_t topR     = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[1]);
  vector_t bottomL  = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[2]);
  vector_t bottomM  = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[3]);
  vector_t bottomR  = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[4]);


  double front_middle_x = (bottomL[0] + bottomR[0])/2.0;
  double front_middle_y = (bottomL[1] + bottomR[1])/2.0;

  double originX = 0.001*(0.8*front_middle_x + 1.2*bottomM[0])/2.0;
  double originY = 0.001*(front_middle_y + bottomM[1])/2.0;
  double theta1, theta2, theta;
  //if( fabs(bottomL[0] - bottomR[0]) != 0.0 ) theta1 = atan2( (bottomL[1] - bottomR[1]) , (bottomL[0] - bottomR[0]));
  //else theta1 = M_PI / 2.0;

  if( fabs(topR[0] - topL[0]) != 0.0 ) theta = atan2( (topR[1] - topL[1]) , (topR[0] - topL[0]));
  else theta = M_PI / 2.0;

  //theta = (theta1 + theta2)/2.0;

  signalOutput_->length (3);
  signalOutput_[0] = originX;
  signalOutput_[1] = originY;
  signalOutput_[2] = theta;

  signalTimestampOutput_->length (2);
  signalTimestampOutput_[0] = msg->body_markers.tv_sec;
  signalTimestampOutput_[1] = msg->body_markers.tv_usec;

  //  LOG ()
  //    << "-> "
  //    << originX << " | "
  //    << originY << " | "
  //    << theta * 180. / M_PI << " deg" << std::endl;
}

void
ChairTracker::simulateSignal ()
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::posix_time::ptime ptime_t;

  static const double mean = 0.;
  static const double standard_deviation = 1.;
  boost::normal_distribution<> dist (mean, standard_deviation);

  boost::variate_generator<boost::mt19937&,
    boost::normal_distribution<> >
    die (chair::gen, dist);

  signalOutput_->length (3);
  for (unsigned i = 0; i < 3; ++i)
    signalOutput_[i] =  die ();

  signalTimestampOutput_->length (2);

  ptime_t time =
    boost::posix_time::microsec_clock::universal_time ();
  int64_t sec = time.time_of_day ().total_seconds();
  int64_t usec =
    time.time_of_day ().total_microseconds () - sec * 1000000;

  typedef boost::numeric::converter<double, int64_t> Int64_t2Double;

  signalTimestampOutput_[0] = Int64_t2Double::convert (sec);
  signalTimestampOutput_[1] = Int64_t2Double::convert (usec);
}
