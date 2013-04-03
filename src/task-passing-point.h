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

/*! \file task-passing-point.h                                                                                                                                                                                 
  \brief Defines a task based on time the initial and final position/velocity 
  respecting time constraints.
*/


#ifndef __sot_oscar_TaskPassingPoint_H__
#define __sot_oscar_TaskPassingPoint_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_passing_point_EXPORTS)
#    define SOTTASKPASSINGPOINT_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKPASSINGPOINT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKPASSINGPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/task.hh>

namespace dynamicgraph {
  namespace sot {
    namespace oscar {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      class SOTTASKPASSINGPOINT_EXPORT TaskPassingPoint
	:public ::dynamicgraph::sot::dyninv::TaskDynPD
	,public ::dynamicgraph::EntityHelper<TaskPassingPoint>
	{

	public:
	  /*! Constructor                                                                                                                                                                               
	    @param name: Name of the task (string)                                                                                                             
	  */
	  TaskPassingPoint( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  typedef TaskPassingPoint EntityClassName;
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(velocityDes, ml::Vector);
	  DECLARE_SIGNAL_IN(duration, double);
	  DECLARE_SIGNAL_IN(initialTime, double);

	  DECLARE_SIGNAL_OUT(velocityCurrent, ml::Vector);
	  DECLARE_SIGNAL_OUT(velocityDesired, ml::Vector);


	protected:
	  /* --- COMPUTATION --- */
	  dg::sot::VectorMultiBound& computeTaskSOUT( dg::sot::VectorMultiBound& task, int iter );

	private:
	  ml::Vector previousTask;

	}; // class  TaskPassingPoint

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_oscar_TaskPassingPoint_H__
