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

#ifndef EVART_TO_CORBA_TRACKED_BODY_HH
# define EVART_TO_CORBA_TRACKED_BODY_HH
# include <string>

# include "application.hh"

class Application;

class TrackedBody
{
public:
  TrackedBody (Application& app,
	       const std::string& signalName,
	       unsigned bodyId,
	       unsigned nbMarkers);
  ~TrackedBody ();

  void writeSignal (const evas_msg_t* msg);
  virtual void computeSignal (const evas_msg_t* msg) = 0;

  unsigned nbMarkers () const
  {
    return nbMarkers_;
  }

protected:
  dynamicGraph::DoubleSeq_var signalOutput_;
private:
  Application& application_;
  unsigned bodyId_;
  unsigned signalRank_;
  unsigned nbMarkers_;
};

class WaistTracker : public TrackedBody
{
public:
  WaistTracker (Application& app);
  ~WaistTracker ();

  virtual void computeSignal (const evas_msg_t* msg);
};

#endif //! EVART_TO_CORBA_TRACKED_BODY_HH
