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

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

extern "C"
{
#include <evart-client.h>
}

#include "corba-connection.hh"
#include "corba-signal.hh"

//#define ENABLE_DEBUG
#include "debug.hh"

#include "application.hh"

#include "tracked-body-factory.hh"

bool exiting = false;

namespace
{
  void displayCorbaInfo (const std::string& dgServiceName,
			 const std::string& dgServiceKind)
  {
    boost::format corbaInfo
      (
       "Corba information:\n"
       "\tdynamic-graph service name: %1%\n"
       "\tdynamic-graph service kind: %2%\n"
       );
    corbaInfo
      % dgServiceName
      % dgServiceKind;
    std::cout << corbaInfo.str () << std::endl;
  }

  int handler (const evas_msg_t* msg, void* data)
  {
    Application* app = (Application*) data;
    app->handler (msg);
    return exiting;
  }

  void signalHandler (int)
  {
    exiting = true;
  }
} // end of anonymous namespace.




Application::Application (int argc, char* argv[])
  : corba_ (argv[0]),
    serverPtr_ (),
    evartHost_ (),
    evartPort_ (),
    mode_ (MODE_TRACKING),
    debug_ (false)
{
  LOG () << "Initialize" << std::endl;

  namespace po = boost::program_options;
  po::options_description desc ("Allowed options");
  desc.add_options ()
    ("dg-service-name,n",
     po::value<std::string>()->default_value ("signal"),
     "dynamic-graph CORBA service name")
    ("dg-service-kind,k",
     po::value<std::string>()->default_value (""),
     "dynamic-graph service kind")

    ("evart-host",
     po::value<std::string>()->default_value (EVAS_STREAM_HOST),
     "evart stream server hostname")

    ("evart-port",
     po::value<short unsigned>()->default_value (EVAS_STREAM_PORT),
     "evart stream server hostname")

    ("list-bodies,l", "list bodies and exit")

    ("list-trackers,L", "list trackers and exit")

    ("simulation,s", "enable simulation mode")

    ("debug,d", "enable debug mode")

    ("bodies,b",
     po::value<std::vector<std::string> >()->composing(),
     "tracked bodies list")

    ("help,h", "produce help message")
    ;

  po::variables_map vm;
  po::store (po::parse_command_line (argc, argv, desc), vm);
  po::notify (vm);

  if (vm.count ("help"))
    {
      std::stringstream ss;
      ss << "Usage: " << argv[0] << " [options]" << std::endl
	 << desc << std::endl
	 << "Report bugs to <hpp@laas.fr>" << std::endl;
      throw PrintUsage (ss.str ());
    }

  if (vm.count ("list-bodies")
      + vm.count ("list-trackers")
      + vm.count ("simulation") > 1)
    throw std::runtime_error ("incompatible options");

  if (vm.count ("list-bodies"))
    mode_ = MODE_BODY_LIST;
  else if (vm.count ("list-trackers"))
    mode_ = MODE_TRACKERS_LIST;
  else if (vm.count ("simulation"))
    mode_ = MODE_TRACKING_SIMULATION;

  debug_ = vm.count ("debug") != 0;

  std::string dgServiceName = vm["dg-service-name"].as<std::string> ();
  std::string dgServiceKind = vm["dg-service-kind"].as<std::string> ();

  evartHost_ = vm["evart-host"].as<std::string> ();
  evartPort_ = vm["evart-port"].as<short unsigned> ();

  setupSignalHandler ();
  connectToMotionCapture ();

  // Only start CORBA during tracking.
  if (mode_ == MODE_TRACKING || mode_ == MODE_TRACKING_SIMULATION)
    {
      displayCorbaInfo (dgServiceName, dgServiceKind);

      LOG () << "Connecting to CORBA server..." << std::endl;
      CORBA::Object_ptr corba_obj =
	corba_.connectToServant (dgServiceName, dgServiceKind);
      LOG () << "Connected to CORBA server." << std::endl;

      try
	{
	  serverPtr_ = dynamicGraph::CorbaSignal::_narrow (corba_obj);

	  if (CORBA::is_nil (serverPtr_))
	    throw std::runtime_error ("failed to connect to the server.");
	}
      catch (CORBA::TRANSIENT& exception)
	{
	  std::cerr
	    << "Failed to connect to dynamic-graph." << std::endl
	    << "1. Double check that the server is started." << std::endl
	    << "2. Does the server and client version match?" << std::endl
	    << std::endl
	    << "Minor code: " << exception.minor () << std::endl;
	  throw;
	}

      if (vm.count ("bodies"))
	initializeTrackedBodies
	  (vm["bodies"].as<std::vector<std::string> > ());
      else
	{
	  std::cerr << "nothing to track, exiting." << std::endl;
	  exiting = true;
	}
    }
}

void
Application::connectToMotionCapture ()
{
  if (mode () != MODE_TRACKING)
    return;

  LOG () << "Connect to motion capture" << std::endl;

  evas_setport (evartPort_);
  evas_sethost (evartHost_.c_str ());

  const evas_body_list_t* bodyList = evas_body_list ();
  if (!bodyList)
    throw std::runtime_error ("failed to retrieve body list");

  LOG () << "Number of bodies: " << bodyList->nbodies << std::endl;
  if (bodyList->nbodies == 0)
    throw std::runtime_error ("no bodies are being streamed");
}

void
Application::listBodies ()
{
  const evas_body_list_t* bodyList = evas_body_list ();
  if (!bodyList)
    throw std::runtime_error ("failed to retrieve body list");

  unsigned nbodies = bodyList->nbodies;
  std::cout << "Number of bodies: " << nbodies << std::endl;

  if (!nbodies)
    return;

  boost::format fmt ("* %1%[%2%] - %3% marker(s)");
  boost::format fmtMarker ("\t- %1%");
  for (unsigned i = 0; i < nbodies; ++i)
    {
      const evas_body_markers_list_t* bodyMarkers =
	evas_body_markers_list (i);
      fmt
	% bodyMarkers->name
	% bodyMarkers->index
	% bodyMarkers->nmarkers;
      std::cout << fmt.str () << std::endl;
      for (int j = 0; j < bodyMarkers->nmarkers; ++j)
	{
	  fmtMarker % bodyMarkers->markers[j];
	  std::cout << fmtMarker.str () << std::endl;
	}
    }
}

void
Application::initializeTrackedBodies (const std::vector<std::string>& trackers)
{
  BOOST_FOREACH(const std::string& str, trackers)
    {
      try
	{
	  boost::shared_ptr<TrackedBody> ptr = trackedBodyFactory (str, *this);
	  addTrackedBody (ptr);
	  boost::format fmt ("tracker %1% initialized");
	  fmt % str;
	  std::cout << fmt.str () << std::endl;
	}
      catch (std::runtime_error& e)
	{
	  boost::format fmt ("failed to initialize tracker %1%");
	  fmt % str;
	  std::cerr << fmt.str () << std::endl;
	}
    }

  if (trackedBodies_.empty ())
    {
      std::cerr << "no running trackers, exiting..." << std::endl;
      exiting = true;
    }
}

void
Application::listTrackers ()
{
  ::listTrackers ();
}

void
Application::process ()
{
  LOG () << "Start processing" << std::endl;

  if (mode () == MODE_BODY_LIST)
    listBodies ();
  else if (mode () == MODE_TRACKERS_LIST)
    listTrackers ();
  else if (mode () == MODE_TRACKING_SIMULATION)
    {
      std::cout << "Start simulation" << std::endl;
      while (!exiting)
	{
	  BOOST_FOREACH (boost::shared_ptr<TrackedBody> e, trackedBodies_)
	    {
	      e->simulateSignal ();
	      e->writeSignal ();
	    }
	}
    }
  else
    {
      // Unpoll as many messages as possible to avoid receiving
      // obsolete message kept in the buffer.
      if (!exiting)
	{
	  evas_msg_t msg;
	  evas_sethandler (0, 0);
	  while (evas_recv (&msg, 0.001))
	    {}
	}

      std::cout << "Start processing" << std::endl;
      while (!exiting)
	{
	  evas_sethandler (::handler, this);
	  evas_listen ();
	  sleep (1);
	}
    }
}

Application::~Application ()
{
  LOG () << "Destruct" << std::endl;
}

void
Application::handler (const evas_msg_t* msg)
{

  if (msg->type == EVAS_BODY_MARKERS)
    {
      BOOST_FOREACH (boost::shared_ptr<TrackedBody> e, trackedBodies_)
	{
	  if (e && msg->body_markers.nmarkers - e->nbMarkers () == 0)
	    {
	      for (int i = 0; i < msg->body_markers.nmarkers; ++i)
		for (unsigned j = 0; j < 3; ++j)
		  if (msg->body_markers.markers[i][j] == EVAS_EMPTY)
		    {
#ifdef ENABLE_DEBUG
		      static const char* axisMessage[] = {"x", "y", "z"};
		      boost::format fmt ("marker %1% (%2%) lost");
		      fmt % i % axisMessage[j];
		      LOG () << fmt.str () << std::endl;
#endif // ENABLE_DEBUG
		    }

	      e->computeSignal (msg);
	      e->logRawData (msg);
	      e->writeSignal ();
	      return;
	    }
	}
      LOG () << "unprocessed message" << std::endl;
    }
}

void
Application::setupSignalHandler ()
{
  struct sigaction sigAction;
  sigAction.sa_handler = signalHandler;
  sigemptyset(&sigAction.sa_mask);
  sigAction.sa_flags = 0;
  sigaction (SIGINT, &sigAction, 0);
  sigaction (SIGHUP, &sigAction, 0);
  sigaction (SIGTERM, &sigAction, 0);
}
