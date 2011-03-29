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

#include <iostream>
#include <stdexcept>

#include <evart-client.h>

#include "corba-connection.hh"
#include "corba-signal.hh"

#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
# define LOG() std::cerr
#else
# define LOG() if (0) std::cerr
#endif // ENABLE_DEBUG

int handler (const evas_msg_t* msg, void* data);

struct Application
{
  Application (const char* argv0)
    : corba_ (argv0)
  {
    LOG () << "Initialize" << std::endl;

    CORBA::Object_ptr corba_obj = corba_.connectToServant ("signal", "");

    try
      {
	serverPtr_ = dynamicGraph::CorbaSignal::_narrow (corba_obj);
      }
    catch (CORBA::TRANSIENT& exception)
      {
	std::cerr << "Failed to connect to the stack of tasks." << std::endl
		  << "1. Double check that the server is started." << std::endl
		  << "2. Does the server and client version match?" << std::endl
		  << std::endl
		  << "Minor code: " << exception.minor () << std::endl;
	throw;
      }

    if (CORBA::is_nil (serverPtr_))
      throw std::runtime_error ("failed to connect to the server.");

    // Create CORBA signal.
  }

  void setup ()
  {
    LOG () << "Setup" << std::endl;
    if (evas_acquire (EVAS_ON))
      throw std::runtime_error ("failed to initialize");

    const evas_body_list_t* bodyList = evas_body_list ();
    if (!bodyList)
      throw std::runtime_error ("failed to retrieve body list");

    LOG () << "Number of bodies: " << bodyList->nbodies << std::endl;
    if (bodyList->nbodies != 1)
      throw std::runtime_error ("bad number of bodies");

    const evas_body_markers_list_t* markersList = evas_body_markers_list (0);
    if (!markersList)
      throw std::runtime_error ("failed to retrieve markers list");

    LOG () << "Number of markers: " << markersList->nmarkers << std::endl;
    if (markersList->nmarkers != 4)
      throw std::runtime_error ("bad number of markers in body 0");

    evas_sethandler (::handler, this);

    if (evas_body_markers (0, EVAS_ON))
      throw std::runtime_error ("failed to enable tracking of body 0");
  }

  void start ()
  {
    LOG () << "Start" << std::endl;
    evas_listen ();
  }

  ~Application ()
  {
    LOG () << "Destruct" << std::endl;
    if (evas_acquire (EVAS_OFF))
      std::cerr << "failed to stop" << std::endl;
  }

  void writeWaistFrame ()
  {
    // waistFrame = new nsCorba::DoubleSeq;
    // waistFrame->length (3);
    // serverPtr_->writeOutputVectorSignal();
  }

  void handler (const evas_msg_t* msg)
  {
    if (msg->type == EVAS_BODY_MARKERS && msg->body_markers_list.nmarkers == 4)
      {
	//computeWaistFrame (msg);
	//writeWaistFrame (msg);

	// std::cout << msg->body_markers.markers[i][0] << " ";
	// std::cout << msg->body_markers.markers[i][1] << " ";
	// std::cout << msg->body_markers.markers[i][2] << std::endl;
      }
  }

  CorbaConnection corba_;
  dynamicGraph::CorbaSignal_var serverPtr_;
};

int handler (const evas_msg_t* msg, void* data)
{
  Application* app = (Application*) data;
  app->handler (msg);
  return 0;
}

int main (int, const char* argv[])
{
  try
    {
      Application app (argv[0]);
      app.setup ();
      app.start ();
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what () << std::endl;
      return 1;
    }
}
