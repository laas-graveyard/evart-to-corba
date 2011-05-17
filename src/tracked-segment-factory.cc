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
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>

#include "tracked-body-factory.hh"

// Include all segments
#include "segment-trackers/segment-waist-tracker.hh"

class Application;

typedef boost::function<boost::shared_ptr<TrackedSegment> (Application&)>
cloneFunction_t;

std::map<std::string, cloneFunction_t> trackerRegisterSegment;

#define REGISTER_TRACKER(CLASS)						\
  trackerRegisterSegment[CLASS::BODY_NAME + "::" + CLASS::SEGMENT_NAME]	\
  = CLASS::clone

void initializeSegmentTrackerRegister ()
{
  REGISTER_TRACKER (WaistTrackerSegment);
}

boost::shared_ptr<TrackedSegment>
trackedSegmentFactory (const std::string& name,
		       Application& app)
{
  if (trackerRegisterSegment.empty ())
    initializeSegmentTrackerRegister ();

  std::map<std::string, cloneFunction_t>::const_iterator it =
    trackerRegisterSegment.find (name);
  if (it == trackerRegisterSegment.end ())
    {
      boost::format fmt ("failed to find tracker '%1%'");
      fmt % name;
      throw std::runtime_error (fmt.str ());
    }
  return it->second (app);
}

void listSegmentTrackers ()
{
  if (trackerRegisterSegment.empty ())
    initializeSegmentTrackerRegister ();

  typedef std::pair<std::string, cloneFunction_t> elt_t;

  std::cout << "Supported segment trackers:" << std::endl;
  if (trackerRegisterSegment.empty ())
    std::cout << "<no tracker>" << std::endl;
  BOOST_FOREACH (const elt_t& e, trackerRegisterSegment)
    {
      boost::format fmt ("* %1%");
      fmt % e.first;
      std::cout << fmt.str () << std::endl;
    }
}
