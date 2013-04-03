/*
 * Copyright 2012, Oscar E. Ramos Ponce, Nicolas Mansard, LAAS-CNRS
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

#ifndef __sot_oscar_SolverMotionReduced_H__
#define __sot_oscar_SolverMotionReduced_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (solver_motion_reduced_EXPORTS)
#    define SOTSOLVERMOTIONREDUCED_EXPORT __declspec(dllexport)
#  else
#    define SOTSOLVERMOTIONREDUCED_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSOLVERMOTIONREDUCED_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/stack-template.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <soth/HCOD.hpp>
#include <Eigen/QR>
//#include <Eigen/SVD>
#include <sot-dyninv/col-piv-qr-solve-in-place.h>

namespace dynamicgraph {
  namespace sot {
    namespace oscar {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTSOLVERMOTIONREDUCED_EXPORT SolverMotionReduced
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<SolverMotionReduced>
	,public sot::Stack< dyninv::TaskDynPD >
	{

	public:
	  /* --- CONSTRUCTOR ---- */
	  SolverMotionReduced( const std::string & name );

	  /* --- STACK INHERITANCE --- */
	  typedef sot::Stack<dyninv::TaskDynPD> stack_t;
	  using stack_t::TaskDependancyList_t;
	  using stack_t::StackIterator_t;
	  using stack_t::StackConstIterator_t;
	  using stack_t::stack;

	  virtual TaskDependancyList_t getTaskDependancyList( const dyninv::TaskDynPD& task );
	  virtual void addDependancy( const TaskDependancyList_t& depList );
	  virtual void removeDependancy( const TaskDependancyList_t& depList );
	  virtual void resetReady( void );

	  /* --- ENTITY INHERITANCE --- */
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  /* --- SIGNALS --- */
	  DECLARE_SIGNAL_IN(matrixInertia,ml::Matrix);
	  DECLARE_SIGNAL_IN(inertiaSqroot,ml::Matrix);
	  DECLARE_SIGNAL_IN(inertiaSqrootInv,ml::Matrix);
	  DECLARE_SIGNAL_IN(velocity,ml::Vector);
	  DECLARE_SIGNAL_IN(dyndrift,ml::Vector);
	  DECLARE_SIGNAL_IN(damping,double);
	  DECLARE_SIGNAL_IN(breakFactor,double);
	  DECLARE_SIGNAL_IN(posture,ml::Vector);
	  DECLARE_SIGNAL_IN(position,ml::Vector);

	  DECLARE_SIGNAL_IN(correct3d,ml::Vector);

	  DECLARE_SIGNAL_OUT(precompute,int);

	  DECLARE_SIGNAL_OUT(inertiaSqrootOut,ml::Matrix);
	  DECLARE_SIGNAL_OUT(inertiaSqrootInvOut,ml::Matrix);

	  DECLARE_SIGNAL_OUT(sizeForcePoint,int);
	  DECLARE_SIGNAL_OUT(sizeForceSpatial,int);
	  DECLARE_SIGNAL_OUT(sizeConfiguration,int);

	  DECLARE_SIGNAL_OUT(Jc,ml::Matrix);
	  DECLARE_SIGNAL_OUT(forceGenerator,ml::Matrix);
	  DECLARE_SIGNAL_OUT(freeMotionBase,ml::Matrix);
	  DECLARE_SIGNAL_OUT(freeForceBase,ml::Matrix);
	  DECLARE_SIGNAL_OUT(driftContact,ml::Vector);
	  DECLARE_SIGNAL_OUT(sizeMotion,int);
	  DECLARE_SIGNAL_OUT(sizeActuation,int);

	  DECLARE_SIGNAL_OUT(solution,ml::Vector);
	  DECLARE_SIGNAL_OUT(reducedControl,ml::Vector);
	  DECLARE_SIGNAL_OUT(reducedForce,ml::Vector);
	  DECLARE_SIGNAL_OUT(acceleration,ml::Vector);
	  DECLARE_SIGNAL_OUT(forces,ml::Vector);
	  DECLARE_SIGNAL_OUT(torque,ml::Vector);

	  DECLARE_SIGNAL_OUT(forcesNormal,ml::Vector);
	  DECLARE_SIGNAL_OUT(activeForces,ml::Vector);

	  /* Temporary time-dependant shared variables. */
	  DECLARE_SIGNAL(Jcdot,OUT,ml::Matrix);

	  /* --- COMMANDS --- */
	  void debugOnce( void );


	  /* --- CONTACT POINTS --- */
	private: 

	  typedef boost::shared_ptr<dynamicgraph::SignalPtr<ml::Matrix,int> > matrixSINPtr;
	  typedef boost::shared_ptr<dynamicgraph::SignalPtr<ml::Vector,int> > vectorSINPtr;
	  typedef boost::shared_ptr<dynamicgraph::Signal<ml::Vector,int> > vectorSOUTPtr;
	  struct Contact
	  {
	    matrixSINPtr jacobianSIN;
	    matrixSINPtr JdotSIN;
	    matrixSINPtr supportSIN;
	    vectorSINPtr correctorSIN;
	    matrixSINPtr corrector3dSIN;
	    vectorSOUTPtr forceSOUT,fnSOUT;
	    int position;
	    std::pair<int,int> range;
	  };
	  typedef std::map< std::string,Contact > contacts_t;
	  contacts_t contactMap;

	public:
	  void addContact( const std::string & name,
			   dynamicgraph::Signal<ml::Matrix,int> * jacobianSignal,
			   dynamicgraph::Signal<ml::Matrix,int> * JdotSignal,
			   dynamicgraph::Signal<ml::Vector,int> * corrSignal,
			   dynamicgraph::Signal<ml::Matrix,int> * contactPointsSignal,
			   dynamicgraph::Signal<ml::Matrix,int> * corr3dSignal );
	  void addContactFromTask( const std::string & taskName, const std::string & contactName );
	  void removeContact( const std::string & name );
	  void dispContacts( std::ostream& os ) const;

	  matrixSINPtr getSupportSIN( const std::string & contacName );


	private:
	  /* --- INTERNAL COMPUTATIONS --- */
	  void refreshTaskTime( int time );
	  void resizeSolver( void );
	  void computeSizesForce( int t );

	  typedef boost::shared_ptr<soth::HCOD> hcod_ptr_t;
	  hcod_ptr_t hsolver;

	  int G_rank;
	  Eigen::ColPivQRSolveInPlace X_qr,Gt_qr;

	  // To compute 'psi'
	  //Eigen::ColPivHouseholderQR K_qr;


	  Eigen::MatrixXd Cforce, Cfnormal, Czero;
	  soth::VectorBound bforce, bfnormal, bzero;
	  std::vector< Eigen::MatrixXd > Ctasks;
	  std::vector< soth::VectorBound > btasks;

	  Eigen::MatrixXd BV;

	  /* Force drift = xddot^* - Jdot qdot. */
	  Eigen::VectorXd solution,forceDrift;

	}; // class SolverMotionReduced

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_oscar_SolverMotionReduced_H__
