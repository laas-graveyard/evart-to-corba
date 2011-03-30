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

#ifndef EVART_TO_CORBA_WAIST_TRACKER_HH
# define EVART_TO_CORBA_WAIST_TRACKER_HH
# include "tracked-body.hh"

# include <boost/numeric/ublas/vector.hpp>
# include <boost/numeric/ublas/matrix.hpp>

class WaistTracker : public TrackedBody
{
public:
  WaistTracker (Application& app);
  ~WaistTracker ();

  virtual void computeSignal (const evas_msg_t* msg);
private:
  /// \name Marker id.
  /// \{
  unsigned front_;
  unsigned leftUp_;
  unsigned rightUp_;
  /// \}
};

#endif //! EVART_TO_CORBA_TRACKED_BODY_HH
