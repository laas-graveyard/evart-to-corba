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

#ifndef EVART_TO_CORBA_TRACKED_BODY_HH
# define EVART_TO_CORBA_TRACKED_BODY_HH
# include <string>
# include <fstream>

# include "application.hh"

# define TRACKED_BODY_DECL(CLASS)					\
  public:								\
  static const std::string BODY_NAME;					\
  static boost::shared_ptr<TrackedBody> clone (Application& app)	\
  {									\
    return boost::shared_ptr<TrackedBody> (new CLASS (app));		\
  }									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

# define TRACKED_BODY_IMPL(CLASS, BODYNAME)	\
  const std::string CLASS::BODY_NAME = BODYNAME

class Application;

class TrackedBody
{
public:
  TrackedBody (Application& app,
	       const std::string& signalName,
	       const std::string& bodyName,
	       unsigned nbMarkers);
  ~TrackedBody ();

  void writeSignal ();
  virtual void computeSignal (const evas_msg_t* msg) = 0;
  virtual void simulateSignal () = 0;

  void logRawData (const evas_msg_t* msg);

  unsigned nbMarkers () const
  {
    return nbMarkers_;
  }

protected:
  void logSignal ();

  dynamicGraph::DoubleSeq_var signalOutput_;
  dynamicGraph::DoubleSeq_var signalTimestampOutput_;
private:
  Application& application_;
  std::string bodyName_;
  unsigned bodyId_;
  unsigned signalRank_;
  unsigned signalTimestampRank_;
  unsigned nbMarkers_;

  std::ofstream rawLog_;
  std::ofstream valueLog_;
};

#endif //! EVART_TO_CORBA_TRACKED_BODY_HH
