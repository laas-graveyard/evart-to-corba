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
#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include "application.hh"

int main (int argc, char* argv[])
{
  try
    {
      Application app (argc, argv);
      app.connectToMotionCapture ();

      if (!app.listOnly ())
	{
	  //FIXME: user should be able to change that.
	  boost::shared_ptr<TrackedBody> ptr (new WaistTracker (app));
	  app.addTrackedBody (ptr);
	}

      app.process ();
    }
  catch (PrintUsage& printUsage)
    {
      std::cout << printUsage.usage_ << std::endl;
      return 0;
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what () << std::endl;
      return 1;
    }
  catch(CORBA::Exception& exception)
    {
      boost::format fmt
	("A CORBA exception has been raised (exception name: ``%1%'').");
      fmt % exception._name ();
      std::cerr << fmt.str () << std::endl;
      return 1;
    }
  catch (...)
    {
      std::cerr
	<< "Unexpected exception catched. Aborting..."
	<< std::endl;
      return -1;
    }
  return 0;
}
