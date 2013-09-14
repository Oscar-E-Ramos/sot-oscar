/*
 * Copyright 2012, Oscar E. Ramos Ponce, LAAS-CNRS
 *
 * This file is part of sot-oscar.
 * sot-oscar is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-oscar is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-oscar.  If not, see <http://www.gnu.org/licenses/>.
 */

/*! \file velocity-control.h                                                                                                                                                                                 
  \brief Implements a control to close the loop at the velocity level
*/


#ifndef __sot_oscar_VelocityControl_H__
#define __sot_oscar_VelocityControl_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (velocity_control_EXPORTS)
#    define SOTVELOCITYCONTROL_EXPORT __declspec(dllexport)
#  else
#    define SOTVELOCITYCONTROL_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTVELOCITYCONTROL_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>

namespace dynamicgraph {
  namespace sot {
    namespace oscar {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      class SOTVELOCITYCONTROL_EXPORT VelocityControl
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<VelocityControl>
	{

	public:
	  /*! Constructor
	    @param name: Name of the task (string)
	  */
	  VelocityControl( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  typedef VelocityControl EntityClassName;
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  /* --- SIGNALS --- */
	  DECLARE_SIGNAL_IN( refVelocity,  ml::Vector);
	  DECLARE_SIGNAL_IN( position,  ml::Vector);
	  DECLARE_SIGNAL_IN( dt, double);
	  DECLARE_SIGNAL_IN( kp, double);
	  DECLARE_SIGNAL_IN( ki, double); 

	  DECLARE_SIGNAL_OUT(control, ml::Vector);
	  DECLARE_SIGNAL_OUT(velocity, ml::Vector);

	  /* --- HELPERS --- */
	  //void getMeasuredVelocity(const ml::Vector & mlvelocity);
	  /* double applyVelocityTransferFunction( const ml::Vector & mlvelocity,  */
	  /* 					unsigned int i ); */

	private:
	  ml::Vector mlpositionPrevious;
	  ml::Vector error;
	  ml::Vector errorSum;

	}; // class  VelocityControl

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_oscar_VelocityControl_H__
