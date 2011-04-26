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

#include <map>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>

#include "tracked-body-factory.hh"

// Include all trackers
#include "trackers/waist-tracker.hh"
#include "trackers/three-points-tracker.hh"
#include "trackers/helmet.hh"

class Application;

typedef boost::function<boost::shared_ptr<TrackedBody> (Application&)>
cloneFunction_t;

std::map<std::string, cloneFunction_t> trackerRegister;

#define REGISTER_TRACKER(CLASS)				\
  trackerRegister[CLASS::BODY_NAME] = CLASS::clone

void initializeTrackerRegister ()
{
  REGISTER_TRACKER (WaistTracker);
  REGISTER_TRACKER (ThreePointsTracker);
  REGISTER_TRACKER (Helmet);
}

boost::shared_ptr<TrackedBody> trackedBodyFactory (const std::string& name,
						   Application& app)
{
  if (trackerRegister.empty ())
    initializeTrackerRegister ();

  std::map<std::string, cloneFunction_t>::const_iterator it =
    trackerRegister.find (name);
  if (it == trackerRegister.end ())
    {
      boost::format fmt ("failed to find tracker '%1%'");
      fmt % name;
      throw std::runtime_error (fmt.str ());
    }
  return it->second (app);
}

void listTrackers ()
{
  if (trackerRegister.empty ())
    initializeTrackerRegister ();

  typedef std::pair<std::string, cloneFunction_t> elt_t;

  std::cout << "Supported trackers:" << std::endl;
  if (trackerRegister.empty ())
    std::cout << "<no tracker>" << std::endl;
  BOOST_FOREACH (const elt_t& e, trackerRegister)
    {
      boost::format fmt ("* %1%");
      fmt % e.first;
      std::cout << fmt.str () << std::endl;
    }
}
