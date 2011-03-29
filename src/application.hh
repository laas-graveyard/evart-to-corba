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
# include <evart-client.h>

# include <vector>

# include <boost/shared_ptr.hpp>

# include "corba-connection.hh"
# include "corba-signal.hh"

# include "tracked-body.hh"

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

protected:
  void listBodies ();
private:
  CorbaConnection corba_;
  dynamicGraph::CorbaSignal_var serverPtr_;
  CORBA::Long signalRank_;

  std::string evartHost_;
  unsigned evartPort_;

  std::vector<boost::shared_ptr<TrackedBody> > trackedBodies_;

  bool listOnly_;
};

#endif //! EVART_TO_CORBA_EVART_TO_CORBA_HH
