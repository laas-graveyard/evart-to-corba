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

#ifndef EVART_TO_CORBA_HELMET_TRACKER_HH
# define EVART_TO_CORBA_HELMET_TRACKER_HH
# include "tracked-body.hh"

# include <boost/make_shared.hpp>

# include <boost/numeric/ublas/vector.hpp>
# include <boost/numeric/ublas/matrix.hpp>

class HelmetTracker : public TrackedBody
{
  TRACKED_BODY_DECL (HelmetTracker);
public:
  HelmetTracker (Application& app);
  ~HelmetTracker ();

  virtual void computeSignal (const evas_msg_t* msg);
  virtual void simulateSignal ();
private:
  /// \name Marker id.
  /// \{
  unsigned fr_;
  unsigned fl_;
  unsigned t_;
  unsigned b_;
  /// \}
};

#endif //! EVART_TO_CORBA_HELMET_TRACKER_HH
