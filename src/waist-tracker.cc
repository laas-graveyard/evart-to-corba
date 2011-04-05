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

#include "waist-tracker.hh"
#include "table-tracker.hh"

namespace ublas = boost::numeric::ublas;

typedef ublas::vector<double> vector_t;
typedef ublas::matrix<double> matrix_t;

static const double WAIST_WIDTH = 0.3;
static const double WAIST_HEIGHT = 0.25;

WaistTracker::WaistTracker (Application& app)
  : TrackedBody (app, "waistPosition", 0, 7),
    front_ (0),
    leftUp_ (1),
    rightUp_ (2)
{}

WaistTracker::~WaistTracker ()
{}


namespace
{
  //              m2
  // m1    *******
  // ******      *
  // *        *  *
  // *      *    *
  // *   *       *
  // *************
  // m3p         m2p
  //
  void
  checkWaistPlaneHorizontal (const vector_t& m1, const vector_t& m2)
  {
    vector_t m1p = m1;
    m1p[2] = 0.;
    vector_t m2p = m2;
    m2p[2] = 0.;

    vector_t m1d = m1 - m1p;
    vector_t m2d = m2 - m2p;

    double m1Pitch = acos (m1d[2] / sqrt (m1d[0] * m1d[0] + m1d[1] * m1d[1]));
    double m2Pitch = acos (m2d[2] / sqrt (m2d[0] * m2d[0] + m2d[1] * m2d[1]));

    if (m1Pitch > 0.1 || m2Pitch > 0.1)
      std::cerr << "warning: robot not parallel" << std::endl;
  }

  double computeTheta (const vector_t& m1, const vector_t& m2)
  {
    vector_t dm = m1 - m2;
    return atan2 (dm[1] / sqrt (dm[0] * dm[0] + dm[1] * dm[1]),
		  dm[0] / sqrt (dm[0] * dm[0] + dm[1] * dm[1]));
  }

  matrix_t yaw (const double& yaw)
  {
    matrix_t res (4, 4);
    res.clear ();
    for (unsigned i = 0; i < 4; ++i)
      for (unsigned j = 0; j < 4; ++j)
 	res (i, j) = (i == j) ? 1. : 0.;

    res (0, 0) = cos (yaw);
    res (0, 1) = -sin (yaw);
    res (1, 0) = sin (yaw);
    res (1, 1) = cos (yaw);
    return res;
  }

  matrix_t t (const double& x, const double& y, const double& z)
  {
    matrix_t res (4, 4);
    res.clear ();
    for (unsigned i = 0; i < 4; ++i)
      for (unsigned j = 0; j < 4; ++j)
 	res (i, j) = (i == j) ? 1. : 0.;

    res (0, 3) = x;
    res (1, 3) = y;
    res (2, 3) = z;
    return res;
  }


  double computeOriginX (const double& theta, const vector_t& F)
  {
    matrix_t w_M_f = yaw (theta);
    for (unsigned i = 0; i < 3; ++i)
      w_M_f (i, 3) = F[i] / 1000.;

    matrix_t f_M_wa = t (WAIST_HEIGHT / 2., 0., 0.);
    matrix_t res = ublas::prod (w_M_f, f_M_wa);
    return res (0, 3);
  }

  double computeOriginY (const double& theta, const vector_t& L)
  {
    matrix_t w_M_l = yaw (theta);
    for (unsigned i = 0; i < 3; ++i)
      w_M_l (i, 3) = L[i] / 1000.;

    matrix_t l_M_wa = t (0., WAIST_WIDTH / 2., 0.);
    matrix_t res = ublas::prod (w_M_l, l_M_wa);
    return res (1, 3);
  }

} // end of anonymous namespace.

void
WaistTracker::computeSignal (const evas_msg_t* msg)
{
  vector_t frontUp = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[0]);
  vector_t leftUp = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[1]);
  vector_t rightUp = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[2]);
  vector_t leftBack = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[3]);
  vector_t rightBack = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[4]);
  vector_t frontDown = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[5]);
  vector_t back = ublas::make_vector_from_pointer
    (3, msg->body_markers.markers[6]);

  checkWaistPlaneHorizontal (leftUp, leftBack);
  checkWaistPlaneHorizontal (rightUp, rightBack);
  checkWaistPlaneHorizontal (frontUp, frontDown);

  vector_t leftPlane = leftUp - leftBack;
  vector_t rightPlane = rightUp - rightBack;

  // LOG () << "leftUp: " << leftUp << std::endl;
  // LOG () << "leftBack: " << leftBack << std::endl;
  // LOG () << "rightUp: " << rightUp << std::endl;
  // LOG () << "rightBack: " << rightBack << std::endl;
  // LOG () << "left plane: " << leftPlane << std::endl;
  // LOG () << "left plane(n): " << leftPlane / sqrt (leftPlane[0] * leftPlane[0] + leftPlane[1] * leftPlane[1]) << std::endl;
  // LOG () << "right plane: " << rightPlane << std::endl;

  // Merge two computed thetas.
  double theta1 = computeTheta (leftUp, leftBack);
  double theta2 = computeTheta (rightUp, rightBack);

  if (theta1 != theta1 || theta2 != theta2)
    return;
  double theta = (theta1 + theta2) / 2.;

  double originX = computeOriginX (theta, leftUp);
  double originY = computeOriginY (theta, frontUp);

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
