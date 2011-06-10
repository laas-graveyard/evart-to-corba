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

#include "bar-tracker.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

namespace bar
{
  boost::mt19937 gen;
} // end of namespace bar

TRACKED_BODY_IMPL (BarTracker, "bar");

BarTracker::BarTracker (Application& app)
  : TrackedBody (app, "barPosition", BarTracker::BODY_NAME, 5)
{}

BarTracker::~BarTracker ()
{}

void
BarTracker::computeSignal (const evas_msg_t* msg)
{
/**
  FL  -o-               -o- FR               x/|\
      | |---------------| |o MR                |
  BL  -o-               -o- BR            y<---o
**/

  vector_t frontL  = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[0]);
  vector_t backL   = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[1]);
  vector_t frontR  = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[2]);
  vector_t middleR = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[3]);
  vector_t backR   = ublas::make_vector_from_pointer    (3, msg->body_markers.markers[4]);


  bool abortBackL = false;
  bool abortBackR = false;
  bool abortFrontL = false;
  bool abortFrontR = false;

  for(int i=0;i<2;i++)
  {
      if(backL[i] == EVAS_EMPTY) abortBackL = true;
      if(backR[i] == EVAS_EMPTY) abortBackR = true;
      if(frontL[i] == EVAS_EMPTY) abortFrontL = true;
      if(frontR[i] == EVAS_EMPTY) abortFrontR = true;
  }

  bool abortBack = abortBackL || abortBackR;
  bool abortFront = abortFrontL || abortFrontR;

  double originX_b = 0.001*(backL[0] + backR[0])/2.0;
  double originY_b = 0.001*(backL[1] + backR[1])/2.0;

  double originX_f = 0.001*(frontL[0] + frontR[0])/2.0;
  double originY_f = 0.001*(frontL[1] + frontR[1])/2.0;

  double originX = EVAS_EMPTY;
  double originY = EVAS_EMPTY;
  double theta = 0.0;


  if( fabs(frontL[0] - backL[0]) != 0.0 ) theta = atan2( (frontL[1] - backL[1]) , (frontL[0] - backL[0]));
  else theta = M_PI / 2.0;


  if(!abortBack && !abortFront)
  {
      theta = atan2( (frontR[1]+backR[1])/2.0 - (frontL[1]+backL[1])/2.0 , (frontR[0]+backR[0])/2.0 - (frontL[0]+backL[0])/2.0 );
      originX = (originX_b + originX_f)/2.0;
      originY = (originY_b + originY_f)/2.0;
  }
  if(!abortBack && abortFront)
  {
      theta = atan2( backR[1] - backL[1] , backR[0] - backL[0] );
      originX = originX_b + 0.03 * cos(theta);
      originY = originY_b + 0.03 * sin(theta);
  }
  if(abortBack && !abortFront)
  {
      theta = atan2( frontR[1] - frontL[1] , frontR[0] - frontL[0] );
      originX = originX_f + 0.03 * cos(theta);
      originY = originY_f + 0.03 * sin(theta);
  }

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
BarTracker::simulateSignal ()
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::posix_time::ptime ptime_t;

  static const double mean = 0.;
  static const double standard_deviation = 1.;
  boost::normal_distribution<> dist (mean, standard_deviation);

  boost::variate_generator<boost::mt19937&,
    boost::normal_distribution<> >
    die (bar::gen, dist);

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
