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

#ifndef EVART_TO_CORBA_SEGMENT_LEFT_FOOT_TRACKER_HH
# define EVART_TO_CORBA_SEGMENT_LEFT_FOOT_TRACKER_HH
# include "tracked-segment.hh"

# include <boost/make_shared.hpp>

# include <boost/numeric/ublas/vector.hpp>
# include <boost/numeric/ublas/matrix.hpp>

/// \brief Track HRP-2 left foot.
///
///  0.13m
/// |--X--|
/// |--|--| 0.235m
/// |Y-O--|
/// |-----|
///
/// ankle position X:
/// ---
///  O
/// 0.1m
/// ---
///
/// ankle position Y: |0.08m O 0.06m|

class LeftFootTracker : public TrackedSegment
{
  TRACKED_SEGMENT_DECL (LeftFootTracker);
public:
  LeftFootTracker (Application& app);
  ~LeftFootTracker ();

  virtual void computeSignal (const evas_msg_t* msg);
  virtual void simulateSignal ();
};

#endif //! EVART_TO_CORBA_SEGMENT_LEFT_FOOT_TRACKER_HH
