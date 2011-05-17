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

#ifndef EVART_TO_CORBA_EVART_TO_CORBA_HH
# define EVART_TO_CORBA_EVART_TO_CORBA_HH
# include <stdexcept>

extern "C"
{
# include <evart-client.h>
}

# include <vector>

# include <boost/shared_ptr.hpp>
# include <boost/date_time/date.hpp>
# include <boost/date_time/date_duration_types.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <boost/date_time/posix_time/posix_time_types.hpp>


# include "corba-connection.hh"
# include "corba-signal.hh"

# include "tracked-body.hh"
# include "tracked-segment.hh"

class PrintUsage
{
public:
  PrintUsage (const std::string& usage)
    : usage_ (usage)
  {}

  const std::string usage_;
};

namespace {
  int handler (const evas_msg_t* msg, void* data);
} // end of anonymous namespace.

class TrackedBody;
class TrackedSegment;

enum Modes
  {
    MODE_TRACKING,
    MODE_TRACKING_SIMULATION,
    MODE_BODY_LIST,
    MODE_TRACKERS_LIST
  };

class Application
{
public:
  Application (int argc, char* argv[]);
  void setupSignalHandler ();
  void connectToMotionCapture ();
  void process ();
  ~Application ();
  void writeWaistFrame (const evas_msg_t* msg);
  void computeWaistFrame (const evas_msg_t*);
  void handler (const evas_msg_t* msg);

  dynamicGraph::CorbaSignal_var getServerPointer ()
  {
    return serverPtr_;
  }

  void
  addTrackedBody (boost::shared_ptr<TrackedBody> trackedBody)
  {
    trackedBodies_.push_back (trackedBody);
  }

  void
  addTrackedSegment (boost::shared_ptr<TrackedSegment> trackedSegment)
  {
    trackedSegments_.push_back (trackedSegment);
  }

  Modes mode () const
  {
    return mode_;
  }

  bool debug () const
  {
    return debug_;
  }

protected:
  void initializeTrackedBodies (const std::vector<std::string>& trackers);
  void initializeTrackedSegments (const std::vector<std::string>& segments);

  void listBodies ();
  void listTrackers ();
private:
  CorbaConnection corba_;
  dynamicGraph::CorbaSignal_var serverPtr_;
  CORBA::Long signalRank_;

  std::string evartHost_;
  short unsigned evartPort_;

  std::vector<boost::shared_ptr<TrackedBody> > trackedBodies_;
  std::vector<boost::shared_ptr<TrackedSegment> > trackedSegments_;

  Modes mode_;
  bool debug_;
};

unsigned getBodyIdFromName (const std::string& name);
unsigned getSegmentIdFromName (unsigned bodyId, const std::string& name);


std::vector<int64_t> to_timeval(const boost::posix_time::ptime &t);
std::vector<int64_t> to_timeval(const boost::posix_time::time_duration &d);



#endif //! EVART_TO_CORBA_EVART_TO_CORBA_HH
