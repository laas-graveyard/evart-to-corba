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

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <evart-client.h>

#include "corba-signal.hh"

//#define ENABLE_DEBUG
#include "debug.hh"

#include "application.hh"

TrackedBody::TrackedBody (Application& app,
			  const std::string& signalName,
			  unsigned bodyId,
			  unsigned nbMarkers)
  : signalOutput_ (new dynamicGraph::DoubleSeq),
    application_ (app),
    bodyId_ (bodyId),
    signalRank_ (-1),
    nbMarkers_ (nbMarkers)
{
  // Create CORBA signal.
  signalRank_ =
    application_.getServerPointer ()->createOutputVectorSignal
    (signalName.c_str ());

  // Make sure the body exists and contains the expected number of markers.
   const evas_body_markers_list_t* markersList = evas_body_markers_list (bodyId);
   if (!markersList)
     throw std::runtime_error ("failed to retrieve markers list");

   LOG () << "Number of markers: " << markersList->nmarkers << std::endl;
   if (markersList->nmarkers != nbMarkers)
     {
       boost::format fmt ("bad number of markers in body %1%");
       fmt % bodyId;
       throw std::runtime_error (fmt.str ());
     }

   if (evas_body_markers (bodyId, EVAS_ON))
     {
       boost::format fmt ("failed to enable tracking of body %1%");
       fmt % bodyId;
       throw std::runtime_error (fmt.str ());
     }
}

TrackedBody::~TrackedBody ()
{
  if (evas_body_markers (bodyId_, EVAS_OFF))
    std::cerr << "failed to disable body tracking" << std::endl;
}

void
TrackedBody::writeSignal (const evas_msg_t* msg)
{
  application_.getServerPointer ()->writeOutputVectorSignal
    (signalRank_, signalOutput_);
}
