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

/*! \file velocity-correction.h                                                                                                                                                                                 
  \brief Modifies the input velocity according to some transfer function
  experimentally obtained with the HRP-2 robot for some joints.
*/


#ifndef __sot_oscar_VelocityCorrection_H__
#define __sot_oscar_VelocityCorrection_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (velocity_correction_EXPORTS)
#    define SOTVELOCITYCORRECTION_EXPORT __declspec(dllexport)
#  else
#    define SOTVELOCITYCORRECTION_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTVELOCITYCORRECTION_EXPORT
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


      class SOTVELOCITYCORRECTION_EXPORT VelocityCorrection
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<VelocityCorrection>
	{

	public:
	  /*! Constructor                                                                                                                                                                               
	    @param name: Name of the task (string)                                                                                                             
	  */
	  VelocityCorrection( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  typedef VelocityCorrection EntityClassName;
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  /* --- SIGNALS --- */
	  DECLARE_SIGNAL_IN( velocityIN,  ml::Vector);
	  DECLARE_SIGNAL_OUT(velocityOUT, ml::Vector);

	  /* --- HELPERS --- */
	  double applyVelocityTransferFunction( const ml::Vector & mlvelocity, 
						unsigned int i );

	}; // class  VelocityCorrection

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_oscar_VelocityCorrection_H__
