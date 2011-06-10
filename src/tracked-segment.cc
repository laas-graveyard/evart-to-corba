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

#include <boost/numeric/conversion/converter.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <evart-client.h>

#include "corba-signal.hh"

//#define ENABLE_DEBUG
#include "debug.hh"

#include "application.hh"

static std::string
makeFilename (const std::string& type, const std::string& name)
{
  return (boost::format ("/tmp/evart-to-corba-segment-%1%-%2%.dat")
	  % type % name).str ();
}

TrackedSegment::TrackedSegment (Application& app,
				const std::string& signalName,
				const std::string& bodyName,
				const std::string& segmentName,
				unsigned nbMarkers,
				unsigned nbSegments)
  : signalOutput_ (new dynamicGraph::DoubleSeq),
    signalTimestampOutput_ (new dynamicGraph::DoubleSeq),
    application_ (app),
    segmentName_ (segmentName),
    bodyId_ (
	     (application_.mode () == MODE_TRACKING)
	     ? getBodyIdFromName (bodyName)
	     : -1),
    segmentId_ (
	     (application_.mode () == MODE_TRACKING)
	     ? getSegmentIdFromName (bodyId_, segmentName)
	     : -1),
    signalRank_ (-1),
    signalTimestampRank_ (-1),
    nbMarkers_ (nbMarkers),
    nbSegments_ (nbSegments),
    rawLog_ (makeFilename ("raw", segmentName_).c_str ()),
    valueLog_ (makeFilename ("value", segmentName_).c_str ())
{
  if (application_.debug ())
    {
      rawLog_
	<< "# bodyId | segmentId | sec | usec | x | y | z | rx | ry | rz | l"
	<< std::endl;
      valueLog_
	<< "# bodyId | segmentId | sec | usec | signalValue0 ... signalValueN"
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
      // Make sure the segment exists and contains the expected number of markers.
      const evas_body_markers_list_t* markersList =
	evas_body_markers_list (bodyId_);

      if (!markersList)
	throw std::runtime_error ("failed to retrieve markers list");

      const evas_body_segments_list_t* segmentsList =
	evas_body_segments_list (bodyId_);

      if (!segmentsList)
	throw std::runtime_error ("failed to retrieve segments list");

      LOG () << "Number of markers: " << markersList->nmarkers << std::endl;
      if (markersList->nmarkers - nbMarkers != 0)
	{
	  boost::format fmt
	    ("bad number of markers in segment %1%::%2% (id = %3%::%4%)");
	  fmt % bodyName_ % segmentName_ % bodyId_ % segmentId_;
	  throw std::runtime_error (fmt.str ());
	}

      LOG () << "Number of segments: " << segmentsList->nsegments << std::endl;
      if (segmentsList->nsegments - nbSegments != 0)
	{
	  boost::format fmt
	    ("bad number of segments in segment %1%::%2% (id = %3%::%4%)");
	  fmt % bodyName_ % segmentName_ % bodyId_ % segmentId_;
	  throw std::runtime_error (fmt.str ());
	}


      LOG () << "Enabling tracking for segment id " << segmentId_ << std::endl;
      if (evas_body_segments (bodyId_, EVAS_ON))
	{
	  boost::format fmt ("failed to enable tracking of segment %1% (id = %2%)");
	  fmt % segmentName_ % segmentId_;
	  throw std::runtime_error (fmt.str ());
	}
    }
}

TrackedSegment::~TrackedSegment ()
{
  if (application_.mode () == MODE_TRACKING)
    if (evas_body_segments (bodyId_, EVAS_OFF))
      std::cerr << "failed to disable segment tracking" << std::endl;
}

void
TrackedSegment::logRawData (const evas_msg_t* msg)
{
  if (!application_.debug ())
    return;

  for (unsigned i = 0; i < msg->body_segments.nsegments; ++i)
    {
      boost::format fmt ("%i %i %i %i %i %d %d %d");

      fmt % msg->body_markers.iFrame;
      fmt % msg->body_markers.index;
      fmt % i;
      fmt % msg->body_markers.tv_sec;
      fmt % msg->body_markers.tv_usec;

      for (unsigned j = 0; j < 7; ++j)
	fmt % msg->body_segments.segments[i].pos[j];
      rawLog_ << fmt.str () << std::endl;
    }
}

void
TrackedSegment::logSignal ()
{
  if (!application_.debug ())
    return;

  typedef boost::numeric::converter<int64_t, double> Double2Int64_t;

  valueLog_ << segmentId_ << " ";
  for (unsigned i = 0; i < signalTimestampOutput_->length (); ++i)
    valueLog_
      << Double2Int64_t::convert (signalTimestampOutput_[i])
      << " ";
  for (unsigned i = 0; i < signalOutput_->length (); ++i)
    valueLog_ << signalOutput_[i] << " ";
  valueLog_ << std::endl;
}


void
TrackedSegment::writeSignal ()
{
  logSignal ();
  application_.getServerPointer ()->writeOutputVectorSignal
    (signalRank_, signalOutput_);
  application_.getServerPointer ()->writeOutputVectorSignal
    (signalTimestampRank_, signalTimestampOutput_);
}
