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

unsigned getBodyIdFromName (const std::string& name)
{
  const evas_body_list_t* bodyList = evas_body_list ();
  if (!bodyList)
    throw std::runtime_error ("failed to retrieve body list");

  for (int i = 0; i < bodyList->nbodies; ++i)
    if (name == bodyList->bodies[i])
      return i;

  boost::format fmt ("failed to retrieve the body id associated to '%1%'");
  fmt % name;
  throw std::runtime_error (fmt.str ());
}

std::string makeFilename (const std::string& type, const std::string& name)
{
  return (boost::format ("/tmp/evart-to-corba-%1%-%2%.dat")
	  % type % name).str ();
}

TrackedBody::TrackedBody (Application& app,
			  const std::string& signalName,
			  const std::string& bodyName,
			  unsigned nbMarkers)
  : signalOutput_ (new dynamicGraph::DoubleSeq),
    signalTimestampOutput_ (new dynamicGraph::DoubleSeq),
    application_ (app),
    bodyName_ (bodyName),
    bodyId_ (
	     (application_.mode () == MODE_TRACKING)
	     ? getBodyIdFromName (bodyName)
	     : -1),
    signalRank_ (-1),
    signalTimestampRank_ (-1),
    nbMarkers_ (nbMarkers),
    rawLog_ (makeFilename ("raw", bodyName_).c_str ()),
    valueLog_ (makeFilename ("value", bodyName_).c_str ())
{
  if (application_.debug ())
    {
      rawLog_
	<< "# bodyId | sec | usec | markerId | x | y | z"
	<< std::endl;
      valueLog_
	<< "# bodyId | sec | usec | signalValue0 ... signalValueN"
	<< std::endl;
    }

  // Create CORBA signal.
  signalRank_ =
    application_.getServerPointer ()->createOutputVectorSignal
    (signalName.c_str ());
  signalTimestampRank_ =
    application_.getServerPointer ()->createOutputVectorSignal
    ((boost::format ("%1%Timestamp") % signalName).str ().c_str ());

  if (application_.mode () == MODE_TRACKING)
    {
      // Make sure the body exists and contains the expected number of markers.
      const evas_body_markers_list_t* markersList =
	evas_body_markers_list (bodyId_);

      if (!markersList)
	throw std::runtime_error ("failed to retrieve markers list");

      LOG () << "Number of markers: " << markersList->nmarkers << std::endl;
      if (markersList->nmarkers - nbMarkers != 0)
	{
	  boost::format fmt ("bad number of markers in body %1% (id = %2%)");
	  fmt % bodyName_ % bodyId_;
	  throw std::runtime_error (fmt.str ());
	}

      LOG () << "Enabling tracking for body id " << bodyId_ << std::endl;
      if (evas_body_markers (bodyId_, EVAS_ON))
	{
	  boost::format fmt ("failed to enable tracking of body %1% (id = %2%)");
	  fmt % bodyName_ % bodyId_;
	  throw std::runtime_error (fmt.str ());
	}
    }
}

TrackedBody::~TrackedBody ()
{
  if (application_.mode () == MODE_TRACKING)
    if (evas_body_markers (bodyId_, EVAS_OFF))
      std::cerr << "failed to disable body tracking" << std::endl;
}

void
TrackedBody::logRawData (const evas_msg_t* msg)
{
  if (!application_.debug ())
    return;

  for (int i = 0; i < msg->body_markers.nmarkers; ++i)
    {
      boost::format fmt ("%i %i %i %i %d %d %d");

      fmt % msg->body_markers.iFrame;
      fmt % msg->body_markers.index;
      fmt % msg->body_markers.tv_sec;
      fmt % msg->body_markers.tv_usec;

      for (unsigned j = 0; j < 3; ++j)
	fmt % msg->body_markers.markers[i][j];
      rawLog_ << fmt.str () << std::endl;
    }
}

void
TrackedBody::logSignal ()
{
  if (!application_.debug ())
    return;

  valueLog_ << bodyId_ << " ";
  for (unsigned i = 0; i < signalTimestampOutput_->length (); ++i)
    valueLog_ << signalTimestampOutput_[i] << " ";
  for (unsigned i = 0; i < signalOutput_->length (); ++i)
    valueLog_ << signalOutput_[i] << " ";
  valueLog_ << std::endl;
}


void
TrackedBody::writeSignal ()
{
  logSignal ();
  application_.getServerPointer ()->writeOutputVectorSignal
    (signalRank_, signalOutput_);
  application_.getServerPointer ()->writeOutputVectorSignal
    (signalTimestampRank_, signalTimestampOutput_);
}
