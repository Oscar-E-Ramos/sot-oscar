/*
 * copyright 2012, Oscar E. Ramos Ponce, Nicolas Mansard, LAAS-CNRS
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

//#define VP_DEBUG
#define VP_DEBUG_MODE 50
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
class solver_motion_reduced__INIT
{
public:solver_motion_reduced__INIT( void )
  {
    //dynamicgraph::sot::DebugTrace::openFile("/tmp/motionRed.txt");
  }
};
solver_motion_reduced__INIT solver_motion_reduced_initiator;
#endif //#ifdef VP_DEBUG


#include <sot-oscar/solver-motion-reduced.h>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/factory.h>
#include <boost/foreach.hpp>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/pool.h>
#include <sot-dyninv/mal-to-eigen.h>
#include <soth/HCOD.hpp>
#include <sot-dyninv/mal-to-eigen.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/feature-point6d.hh>
#include <sstream>
#include <soth/Algebra.hpp>
#include <Eigen/QR>

namespace dynamicgraph
{
  namespace sot
  {
    namespace oscar
    {
#include <Eigen/Cholesky>

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SolverMotionReduced,"SolverMotionReduced");


      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */
      SolverMotionReduced::
      SolverMotionReduced( const std::string & name )
	: Entity(name)
	,stack_t()

	,CONSTRUCT_SIGNAL_IN(matrixInertia,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(inertiaSqroot,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(inertiaSqrootInv,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(dyndrift,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(damping,double)
	,CONSTRUCT_SIGNAL_IN(breakFactor,double)
	,CONSTRUCT_SIGNAL_IN(posture,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(position,ml::Vector)
	
	,CONSTRUCT_SIGNAL_IN(correct3d, ml::Vector)

	,CONSTRUCT_SIGNAL_OUT(precompute,int,
			      inertiaSqrootInvSIN << inertiaSqrootSIN )
	,CONSTRUCT_SIGNAL_OUT(inertiaSqrootOut,ml::Matrix,
			      matrixInertiaSIN)
	,CONSTRUCT_SIGNAL_OUT(inertiaSqrootInvOut,ml::Matrix,
			      inertiaSqrootSIN)
	,CONSTRUCT_SIGNAL_OUT(sizeForcePoint,int,
			      precomputeSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeForceSpatial,int,
			      precomputeSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeConfiguration,int,
			      velocitySIN )

	,CONSTRUCT_SIGNAL_OUT(Jc,ml::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(forceGenerator,ml::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(freeMotionBase,ml::Matrix,
			      JcSOUT << inertiaSqrootInvSIN)
	,CONSTRUCT_SIGNAL_OUT(freeForceBase,ml::Matrix,
			      forceGeneratorSOUT << JcSOUT)
	,CONSTRUCT_SIGNAL_OUT(driftContact,ml::Vector,
			      freeMotionBaseSOUT<<JcSOUT )

	,CONSTRUCT_SIGNAL_OUT(sizeMotion,int,
			      freeMotionBaseSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeActuation,int,
			      freeForceBaseSOUT )


	,CONSTRUCT_SIGNAL_OUT(solution,ml::Vector,
			      freeMotionBaseSOUT << inertiaSqrootInvSIN << inertiaSqrootSIN
			      << dyndriftSIN << velocitySIN
			      << dampingSIN << breakFactorSIN
			      << postureSIN << positionSIN)
	,CONSTRUCT_SIGNAL_OUT(reducedControl,ml::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(forces,ml::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(acceleration,ml::Vector,
			      reducedControlSOUT << inertiaSqrootSIN << freeMotionBaseSOUT)
	,CONSTRUCT_SIGNAL_OUT(reducedForce,ml::Vector,
			      forcesSOUT)
	,CONSTRUCT_SIGNAL_OUT(torque,ml::Vector,
			      reducedControlSOUT << JcSOUT << reducedForceSOUT << inertiaSqrootSIN )

	,CONSTRUCT_SIGNAL_OUT(forcesNormal,ml::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(activeForces,ml::Vector,
			      solutionSOUT)


	,CONSTRUCT_SIGNAL(Jcdot,OUT,ml::Matrix)

	,hsolver()

	,Cforce(),Czero(),Cfnormal()
	,bforce(),bzero(),bfnormal()
	,Ctasks(),btasks()
	,solution()
      {
	signalRegistration( matrixInertiaSIN
			    << inertiaSqrootSIN
			    << inertiaSqrootInvSIN
			    << velocitySIN
			    << dyndriftSIN
			    << dampingSIN
			    << breakFactorSIN
			    << postureSIN
			    << positionSIN
			    << correct3dSIN

			    );
	signalRegistration(
			   inertiaSqrootOutSOUT
			   << inertiaSqrootInvOutSOUT
			   << JcSOUT
			   << freeMotionBaseSOUT
			   << freeForceBaseSOUT
			   << driftContactSOUT

			   << sizeActuationSOUT
			   << sizeMotionSOUT
			   << sizeForceSpatialSOUT
			   << sizeForcePointSOUT
			   << forceGeneratorSOUT
			   << solutionSOUT
			   << reducedControlSOUT
			   << forcesSOUT
			   << accelerationSOUT
			   << reducedForceSOUT
			   << torqueSOUT

			   << JcdotSOUT
			   );
	signalRegistration( forcesNormalSOUT << activeForcesSOUT );

	inertiaSqrootInvSIN.plug( &inertiaSqrootInvOutSOUT );
	inertiaSqrootSIN.plug( &inertiaSqrootOutSOUT );

	/* Command registration. */
	boost::function<void(SolverMotionReduced*,const std::string&)> f_addContact
	  =
	  boost::bind( &SolverMotionReduced::addContact,_1,_2,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<ml::Vector, int>*)NULL,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL );

	addCommand("addContact",
		   makeCommandVoid1(*this,f_addContact,
				    dyninv::docCommandVoid1("create the contact signals, unpluged.",
						    "string")));
	addCommand("addContactFromTask",
		   makeCommandVoid2(*this,&SolverMotionReduced::addContactFromTask,
				    dyninv::docCommandVoid2("Add a contact from the named task. Remmeber to plug __p.",
						    "string(task name)","string (contact name)")));
	addCommand("rmContact",
		   makeCommandVoid1(*this,&SolverMotionReduced::removeContact,
				    dyninv::docCommandVoid1("remove the contact named in arguments.",
						    "string")));
	addCommand("dispContacts",
		   makeCommandVerbose(*this,&SolverMotionReduced::dispContacts,
				      dyninv::docCommandVerbose("Guess what?")));
	addCommand("debugOnce",
		   makeCommandVoid0(*this,&SolverMotionReduced::debugOnce,
				    dyninv::docCommandVoid0("open trace-file for next iteration of the solver.")));

	//sot::ADD_COMMANDS_FOR_THE_STACK;
	addCommand("dispStack",
		   makeCommandVerbose(*this,&stack_t::display,
				      dyninv::docCommandVerbose("Guess what?")));
	addCommand("up",
		   makeCommandVoid1(*this, (void (EntityClassName::*)(const std::string&)) &stack_t::upByTaskName,		
				    dyninv::docCommandVoid1("Up the named task.", 
						    "string (task name)"))); 
	addCommand("down",						
		   makeCommandVoid1(*this, (void (EntityClassName::*)(const std::string&)) &stack_t::downByTaskName,		
				    dyninv::docCommandVoid1("Down the named task.", 
						    "string (task name)"))); 
	addCommand("push",						
		   makeCommandVoid1(*this, (void (EntityClassName::*)(const std::string&)) &stack_t::pushByTaskName,		
				    dyninv::docCommandVoid1("Push the named task.", 
						    "string (task name)"))); 
	addCommand("rm",						
		   makeCommandVoid1(*this, (void (EntityClassName::*)(const std::string&)) &stack_t::removeByTaskName,		
				    dyninv::docCommandVoid1("Remove the named task.", 
						    "string (task name)"))); 
	addCommand("clear",						
		   makeCommandVoid0(*this, (void (EntityClassName::*)(void)) &stack_t::clear,			
				    dyninv::docCommandVoid0("Clear the stack from all task."))); 
	addCommand("pop",						
		   makeCommandVoid0(*this, (void (EntityClassName::*)(void)) &stack_t::pop0,			
				    dyninv::docCommandVoid0("Remove the last (lowest) task of the stack."))); 
									
	addCommand("setSize",						
		   makeCommandVoid1(*this, (void (EntityClassName::*)(const int&)) &stack_t::defineNbDof,		
				    dyninv::docCommandVoid1("??? TODO",		
						    "int")));		
	addCommand("getSize",						
		   makeDirectGetter(*this,&nbDofs,			
				    dyninv::docDirectGetter("size","int")));
	  
	  
      }

      /* ----------------------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* ----------------------------------------------------------------------- */

      void SolverMotionReduced::
      debugOnce( void )
      {
	dg::sot::DebugTrace::openFile("/tmp/motionRed.txt");
	//hsolver->debugOnce();
      }

      /* --------------------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      SolverMotionReduced::TaskDependancyList_t SolverMotionReduced::
      getTaskDependancyList( const dyninv::TaskDynPD& task )
      {
	TaskDependancyList_t res;
	res.push_back( &task.taskSOUT );
	res.push_back( &task.jacobianSOUT );
	res.push_back( &task.JdotSOUT );
	return res;
      }
      void SolverMotionReduced::
      addDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { solutionSOUT.addDependency( *sig ); }
      }
      void  SolverMotionReduced::
      removeDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { solutionSOUT.removeDependency( *sig ); }
      }
      void SolverMotionReduced::
      resetReady( void )
      {
	solutionSOUT.setReady();
      }

      /* --------------------------------------------------------------------- */
      /* --- CONTACT LIST ---------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      /* TODO: push this method directly in signal. */
      static std::string signalShortName( const std::string & longName )
      {
	std::istringstream iss( longName );
	const int SIZE = 128;
	char buffer[SIZE];
	while( iss.good() )
	  { iss.getline(buffer,SIZE,':'); }
	return std::string( buffer );
      }

      void SolverMotionReduced::
      addContactFromTask( const std::string & taskName,const std::string & contactName )
      {
	using namespace dynamicgraph::sot;

	dyninv::TaskDynPD & task = dynamic_cast<dyninv::TaskDynPD&> ( g_pool().getEntity( taskName ) );
	assert( task.getFeatureList().size() == 1 );
	BOOST_FOREACH( FeatureAbstract* fptr, task.getFeatureList() )
	  {
	    FeaturePoint6d* f6 = dynamic_cast< FeaturePoint6d* >( fptr );
	    assert( NULL!=f6 );
	    f6->positionSIN.recompute(solutionSOUT.getTime());
	    f6->servoCurrentPosition();
	    f6->FeatureAbstract::selectionSIN = true;
	  }
	addContact( contactName, &task.jacobianSOUT, &task.JdotSOUT,&task.taskVectorSOUT, NULL, NULL );
      }

      void SolverMotionReduced::
      addContact( const std::string & name,
		  Signal<ml::Matrix,int> * jacobianSignal,
		  Signal<ml::Matrix,int> * JdotSignal,
		  Signal<ml::Vector,int> * corrSignal,
		  Signal<ml::Matrix,int> * contactPointsSignal,
		  Signal<ml::Matrix,int> * corr3dSignal )
      {
	if( contactMap.find(name) != contactMap.end())
	  {
	    std::cerr << "!! Contact " << name << " already exists." << std::endl;
	    return;
	  }

	contactMap[name].jacobianSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( jacobianSignal,
			    "sotMotionRedWB("+getName()+")::input(matrix)::_"+name+"_J" ) );
	signalRegistration( *contactMap[name].jacobianSIN );
	JcSOUT.addDependency( *contactMap[name].jacobianSIN );
	precomputeSOUT.addDependency( *contactMap[name].jacobianSIN );
	JcSOUT.setReady();
	precomputeSOUT.setReady();

	contactMap[name].JdotSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( JdotSignal,
			    "sotMotionRedWB("+getName()+")::input(matrix)::_"+name+"_Jdot" ) );
	signalRegistration( *contactMap[name].JdotSIN );

	contactMap[name].supportSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( contactPointsSignal,
			    "sotMotionRedWB("+getName()+")::input(matrix)::_"+name+"_p" ) );
	signalRegistration( *contactMap[name].supportSIN );
	forceGeneratorSOUT.addDependency( *contactMap[name].supportSIN );

	contactMap[name].correctorSIN
	  = vectorSINPtr( new SignalPtr<ml::Vector,int>
			  ( corrSignal,
			    "sotMotionRedWB("+getName()+")::input(vector)::_"+name+"_x" ) );
	signalRegistration( *contactMap[name].correctorSIN );

	contactMap[name].forceSOUT
	  = vectorSOUTPtr( new Signal<ml::Vector,int>
			   ( "sotMotionRedWB("+getName()+")::output(vector)::_"+name+"_f" ) );
	signalRegistration( *contactMap[name].forceSOUT );

	contactMap[name].fnSOUT
	  = vectorSOUTPtr( new Signal<ml::Vector,int>
			   ( "sotMotionRedWB("+getName()+")::output(vector)::_"+name+"_fn" ) );
	signalRegistration( *contactMap[name].fnSOUT );

	// For the desired dd(x_3) points
	contactMap[name].corrector3dSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( corr3dSignal,
			    "sotMotionRedWB("+getName()+")::input(matrix)::_"+name+"_ddx3" ) );
	signalRegistration( *contactMap[name].corrector3dSIN );
	JcSOUT.addDependency( *contactMap[name].corrector3dSIN );

	// Check out the correct way to do it !!!
	// *contactMap[name].corrector3dSIN(0) = 0.0;
      }

      void SolverMotionReduced::
      removeContact( const std::string & name )
      {
	if( contactMap.find(name) == contactMap.end() )
	  {
	    std::cerr << "!! Contact " << name << " does not exist." << std::endl;
	    return;
	  }

	JcSOUT.removeDependency( *contactMap[name].jacobianSIN );
	precomputeSOUT.removeDependency( *contactMap[name].jacobianSIN );
	forceGeneratorSOUT.removeDependency( *contactMap[name].supportSIN );
	JcSOUT.removeDependency( *contactMap[name].corrector3dSIN );

	JcSOUT.setReady();
	precomputeSOUT.setReady();
	forceGeneratorSOUT.setReady();

	signalDeregistration( signalShortName(contactMap[name].jacobianSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].supportSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].forceSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].fnSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].JdotSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].correctorSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].corrector3dSIN->getName()) );
	contactMap.erase(name);
      }

      void SolverMotionReduced::
      dispContacts( std::ostream& os ) const
      {
	os << "+-----------------\n+   Contacts\n+-----------------" << std::endl;
	// TODO BOOST FOREACH
	for( contacts_t::const_iterator iter=contactMap.begin();
	     iter!=contactMap.end(); ++iter )
	  {
	    os << "| " << iter->first <<std::endl;
	  }
      }

      SolverMotionReduced::matrixSINPtr SolverMotionReduced::getSupportSIN( const std::string & contacName )
      {
	return contactMap[contacName].supportSIN;
      }



      /* ---------------------------------------------------------------------- */
      /* --- INIT SOLVER ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */

      /** Force the update of all the task in-signals, in order to fix their
       * size for resizing the solver.
       */
      void SolverMotionReduced::
      refreshTaskTime( int time )
      {
	// TODO BOOST_FOREACH
	for( StackIterator_t iter=stack.begin();stack.end()!=iter;++iter )
	  {
	    dyninv::TaskDynPD& task = **iter;
	    task.taskSOUT( time );
	  }
      }



     /* --------------------------------------------------------------------- */
      /* --- STATIC INTERNAL ------------------------------------------------- */
      /* --------------------------------------------------------------------- */
      namespace sotSolverMotion
      {
	template< typename D1,typename D2 >
	void preCross( const Eigen::MatrixBase<D1> & M,Eigen::MatrixBase<D2> & Tx )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D1>);
	  assert( Tx.cols()==3 && Tx.rows()==3 && M.size()==3 );
	  Tx( 0,0 ) = 0   ;  Tx( 0,1 )=-M[2];  Tx( 0,2 ) = M[1];
	  Tx( 1,0 ) = M[2];  Tx( 1,1 )= 0   ;  Tx( 1,2 ) =-M[0];
	  Tx( 2,0 ) =-M[1];  Tx( 2,1 )= M[0];  Tx( 2,2 ) = 0   ;
	}

	//soth::VectorBound& vbAssign ( soth::VectorBound& vb, const Eigen::VectorXd& vx )
	template<typename D1,typename D2>
	Eigen::MatrixBase<D1>& vbAssign ( Eigen::MatrixBase<D1>& vb,
					  const Eigen::MatrixBase<D2>& vx )
	{
	  vb.resize(vx.size());
	  for( int i=0;i<vx.size();++i ){ vb[i] = vx[i]; }
	  return vb;
	}
	soth::VectorBound& vbAssign ( soth::VectorBound& vb,
				      const dg::sot::VectorMultiBound& vx )
	{
	  vb.resize(vx.size());
	  for( int c=0;c<vx.size();++c )
	    {
	      if( vx[c].getMode() == dg::sot::MultiBound::MODE_SINGLE )
		vb[c] = vx[c].getSingleBound();
	      else
		{
		  using dg::sot::MultiBound;
		  using namespace soth;
		  const bool binf = vx[c].getDoubleBoundSetup( MultiBound::BOUND_INF ),
		    bsup = vx[c].getDoubleBoundSetup( MultiBound::BOUND_SUP );
		  if( binf&&bsup )
		    {
		      vb[c]
			= std::make_pair( vx[c].getDoubleBound(MultiBound::BOUND_INF),
					  vx[c].getDoubleBound(MultiBound::BOUND_SUP) );
		    }
		  else if( binf )
		    {
		      vb[c] = Bound( vx[c].getDoubleBound(MultiBound::BOUND_INF),
				     Bound::BOUND_INF );
		    }
		  else
		    {
		      assert( bsup );
		      vb[c] = Bound( vx[c].getDoubleBound(MultiBound::BOUND_SUP),
				     Bound::BOUND_SUP );
		    }

		}
	    }
	  return vb;
	}
	template<typename D>
	soth::VectorBound& vbSubstract ( soth::VectorBound& vb,
					 const Eigen::MatrixBase<D> &vx )
	{
	  using namespace soth;
	  assert( vb.size() == vx.size() );
	  for( int c=0;c<vx.size();++c )
	    {
	      const Bound::bound_t & type = vb[c].getType();
	      switch( type )
		{
		case Bound::BOUND_INF:
		case Bound::BOUND_SUP:
		  vb[c] = Bound( type,vb[c].getBound(type)-vx[c] );
		  break;
		case Bound::BOUND_DOUBLE:
		  vb[c] = std::make_pair( vb[c].getBound(Bound::BOUND_INF)-vx[c],
					  vb[c].getBound(Bound::BOUND_SUP)-vx[c] );
		  break;
		case Bound::BOUND_TWIN:
		  vb[c] = vb[c].getBound(type) - vx[c];
		  break;
		case Bound::BOUND_NONE:
		  assert( false &&"This switch should not happen." );
		  break;
		}
	    }
	  return vb;
	}
	template<typename D>
	soth::VectorBound& vbAdd ( soth::VectorBound& vb,
				   const Eigen::MatrixBase<D> &vx )
	{
	  using namespace soth;
	  assert( vb.size() == vx.size() );
	  for( int c=0;c<vx.size();++c )
	    {
	      const Bound::bound_t & type = vb[c].getType();
	      switch( type )
		{
		case Bound::BOUND_INF:
		case Bound::BOUND_SUP:
		  vb[c] = Bound( type,vb[c].getBound(type)+vx[c] );
		  break;
		case Bound::BOUND_DOUBLE:
		  vb[c] = std::make_pair( vb[c].getBound(Bound::BOUND_INF)+vx[c],
					  vb[c].getBound(Bound::BOUND_SUP)+vx[c] );
		  break;
		case Bound::BOUND_TWIN:
		  vb[c] = vb[c].getBound(type) + vx[c];
		  break;
		case Bound::BOUND_NONE:
		  assert( false &&"This switch should not happen." );
		  break;
		}
	    }
	  return vb;
	}

	/* TODO: inherite from JacobiSVD a structure where rank can be computed dynamically. */
	// inline int svdRankDefEval( const Eigen::JacobiSVD<Eigen::MatrixXd >& Msvd,
	// 			   const double threshold = 1e-5 )
	// {
	//   return (Msvd.singularValues().array() > threshold ).count();
	// }
  	// template<typename D2>
	// Eigen::VectorXd svdRankDefSolve( const Eigen::JacobiSVD<Eigen::MatrixXd >& Msvd,
	// 				 const Eigen::MatrixBase<D2>& y,
	// 				 const int rank )
	// {
	//   assert( Msvd.computeU() && Msvd.computeV() );
	//   return
	//     Msvd.matrixV().leftCols(rank) *
	//     Msvd.singularValues().array().head(rank).inverse().matrix().asDiagonal() *
	//     Msvd.matrixU().leftCols(rank).transpose()*y;
	// }
      }

      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      // TODO: the following should be replace by the middleColumn and
      // middleRow functions.
#define COLS(__ri,__rs) leftCols(__rs).rightCols((__rs)-(__ri))
#define ROWS(__ri,__rs) topRows(__rs).bottomRows((__rs)-(__ri))

      ml::Matrix& SolverMotionReduced::
      inertiaSqrootOutSOUT_function( ml::Matrix& mlAsq, int t )
      {
	EIGEN_CONST_MATRIX_FROM_SIGNAL(A,matrixInertiaSIN(t));
	EIGEN_MATRIX_FROM_MATRIX(Asq,mlAsq,A.rows(),A.cols());

	using namespace Eigen;
	sotDEBUG(1) << "A = " << (soth::MATLAB)A << std::endl;

	Asq = A.llt().matrixU();

	/* Asq is such that Asq^T Asq = A. */
	return mlAsq;
      }

      ml::Matrix& SolverMotionReduced::
      inertiaSqrootInvOutSOUT_function( ml::Matrix& mlAsqi,int t)
      {
	EIGEN_CONST_MATRIX_FROM_SIGNAL(Asq,inertiaSqrootSIN(t));
	const int n = Asq.rows();
	EIGEN_MATRIX_FROM_MATRIX(Asqi,mlAsqi,n,n);
	Asqi = Asq.triangularView<Eigen::Upper>().solve( Eigen::MatrixXd::Identity(n,n));

	/* Asqi is such that Asq^-1 = Asqi and Asqi Asqi^T = A^-1. Asqi is upper triangular. */
	return mlAsqi;
      }

      /* This function compute all the input matrix and vector signals that
       * will be needed by the solver. The main objective of this precomputation
       * is to get the size of all the elements. */
      int& SolverMotionReduced::
      precomputeSOUT_function( int& dummy, int t )
      {
	//if( t==1000 )
	//dynamicgraph::sot::DebugTrace::openFile("/tmp/dynred1000.txt");

	/* Precompute the dynamic data. */
	inertiaSqrootInvSIN.recompute(t);
	inertiaSqrootSIN.recompute(t);

	/* Precompute the contact data. */
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    contact.jacobianSIN->recompute(t);
	    contact.JdotSIN->recompute(t);
	    contact.supportSIN->recompute(t);
	    contact.correctorSIN->recompute(t);
	    contact.corrector3dSIN->recompute(t);
	    
	    // if (*contact.corrector3dSIN) // == 0.0)  
	    //   std::cout << "corrector not assigned" << std::endl;

	  }

	/* Precompute the tasks data. */
	for( int i=0;i<(int)stack.size();++i )
	  {
	    stack[i]->taskSOUT.recompute(t);
	    stack[i]->jacobianSOUT.recompute(t);
	  }

	return dummy;
      }

      /* -------------------------------------------------------------------- */
      /* --- SIZES ---------------------------------------------------------- */
      /* -------------------------------------------------------------------- */

      // 6*(total number of bodies in contact)
      int& SolverMotionReduced::
      sizeForceSpatialSOUT_function( int& nf, int t )
      {
	precomputeSOUT(t);

	int nbb=0;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    contact.position = nbb++;
	  }
	nf=nbb*6;

	return nf;
      }

      // 3*(total number of contact points)
      int& SolverMotionReduced::
      sizeForcePointSOUT_function( int& nf, int t )
      {
	precomputeSOUT(t);

	nf=0;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int nfi = (*contact.supportSIN)(t).nbCols()*3;
	    contact.range = std::make_pair(nf,nf+nfi);
	    nf+=nfi;
	  }
	return nf;
      }
 
      int& SolverMotionReduced::
      sizeConfigurationSOUT_function( int& nq, int t )
      {
	nq = velocitySIN(t).size();
	return nq;
      }

      int& SolverMotionReduced::
      sizeMotionSOUT_function( int& nu, int t )
      {
	nu = freeMotionBaseSOUT(t).nbCols();
	return nu;
      }

      int& SolverMotionReduced::
      sizeActuationSOUT_function( int& nphi, int t )
      {
	nphi = freeForceBaseSOUT(t).nbCols();
	return nphi;
      }

      /* -------------------------------------------------------------------- */
      /* --- FORCES MATRICES ------------------------------------------------ */
      /* -------------------------------------------------------------------- */

      /* Compute the Jacobian of the contact bodies, along with the drift: Jc=[Jc1;Jc2;...] */
      ml::Matrix& SolverMotionReduced::
      JcSOUT_function( ml::Matrix& mlJ,int t )
      {
	using namespace Eigen;

	EIGEN_CONST_VECTOR_FROM_SIGNAL(qdot,velocitySIN(t));
	const int &nq= sizeConfigurationSOUT(t),
	  nphi = sizeForceSpatialSOUT(t), nf = sizeForcePointSOUT(t);

       	EIGEN_MATRIX_FROM_MATRIX(J,mlJ,nphi,nq);
	//forceDrift.resize(nphi);
	forceDrift.resize(nf);

	EIGEN_CONST_MATRIX_FROM_SIGNAL(X,forceGeneratorSOUT(t));

	int ncontact = 0;    // Clean this dirty variable!
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    EIGEN_CONST_MATRIX_FROM_SIGNAL(Ji,(*contact.jacobianSIN)(t));
	    EIGEN_CONST_MATRIX_FROM_SIGNAL(Jdoti,(*contact.JdotSIN)(t));
	    EIGEN_CONST_VECTOR_FROM_SIGNAL(corrector,(*contact.correctorSIN)(t));

	    EIGEN_CONST_MATRIX_FROM_SIGNAL(support,(*contact.supportSIN)(t));

	    const int n0 = contact.range.first, n1 = contact.range.second,
	      nbp = support.cols();
	    assert( ((n1-n0) % 3 == 0) && nbp == (n1-n0)/3);

	    const int r = 6*contact.position;
	    assert( Ji.rows()==6 && Ji.cols()==nq );
	    assert( r+6<=J.rows() && r>=0 );
	    J.ROWS(r,r+6) = Ji;

	    //std::cout << "n0: "<< n0 << " - n1: " << n1 << " - nbp: " << std::endl;
	    MatrixXd Xi=X.block(n0, 6*ncontact++, nbp*3, 6); 
	    //std::cout<< "Xi:\n" << Xi << std::endl;

	    MatrixXd wcp; wcp.resize(nbp*3,1);
	    Matrix3d wc_skew;
	    Vector3d wc = Ji.bottomRows(3)*qdot;
	    sotSolverMotion::preCross(wc, wc_skew);
	    
	    //std::cout << wc << std::endl << wc_skew << std::endl;
	    //forceDrift.ROWS(r,r+6) = corrector - Jdoti*qdot;

	    // if (!(*contact.corrector3dSIN))
	    //   std::cout <<"No corrector (precompute)" << std::endl;
	    // else
	    //   std::cout << "corrector found!" << std::endl;

	    EIGEN_CONST_MATRIX_FROM_SIGNAL(corrector3d,(*contact.corrector3dSIN)(t));
	    bool noCorrector3d = false;
	    if ( corrector3d.size()== 3 )
	      {
		double dummySum;
		dummySum = fabs(corrector3d(0)) + fabs(corrector3d(1)) +
		           fabs(corrector3d(2));
		if (dummySum == 0)
		  noCorrector3d = true; // aka, there is no real corrector3d
	      }

	    // If the signal corrector3d is not really set
	    if (noCorrector3d)
	      {
		for( int i=0;i<nbp;++i )
		  wcp.block(i*3, 0, 3, 1) = wc_skew*wc_skew*support.col(i);
		//std::cout << "wcp: " << std::endl << wcp << std::endl;

		forceDrift.ROWS(n0,n0+nbp*3) = Xi*corrector - Xi*Jdoti*qdot + wcp;
		//std::cout<< "forceDrift: " <<std::endl<< forceDrift << std::endl;
	      }

	    // If the signal corrector3d has a value different to zero
	    else
	      {
		// Convert it to a column vector (ddxf)
		VectorXd ddxf; ddxf.resize(corrector3d.size());
		for( int i=0; i<corrector3d.size(); i++)
		  {
		    ddxf(i) = corrector3d(i);
		  }
		//std::cout << ddxf << std::endl;
		forceDrift.ROWS(n0,n0+nbp*3) = -ddxf - Xi*Jdoti*qdot;
		// std::cout << "Xi\n" << Xi*Jdoti*qdot << std::endl;
	      }

	    // //Dirty way of adding the second points explicitly (just to test)
	    // if (n0!=0 && correct3dSIN)
	    //   {
	    // 	EIGEN_CONST_VECTOR_FROM_SIGNAL(ddxf, correct3dSIN(t));
	    // 	// std::cout << "ddxf\n" << ddxf << std::endl;
	    // 	// std::cout << "Xi*Jdoti*qdot\n" << Xi*Jdoti*qdot << std::endl;
	    // 	forceDrift.ROWS(n0,n0+nbp*3) = -ddxf - Xi*Jdoti*qdot;
	    //   }

	  }

	return mlJ;
      }

      /* Compute X such that J = X*Jc      (A*ddq + Jc'*X'*f = tau)
       * X': passes from punctual forces to 6D torques expressed at the body center.
       * X has a diagonal-block structure, not preserved by the
       * current data structure.
       */
      ml::Matrix& SolverMotionReduced::
      forceGeneratorSOUT_function( ml::Matrix& mlX,int t )
      {
	using namespace Eigen;

	const int& nf = sizeForcePointSOUT(t), nphi = sizeForceSpatialSOUT(t);

	EIGEN_MATRIX_FROM_MATRIX(X,mlX,nf,nphi);
	X.fill(0); // This should be avoided to spare computation time.

	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    EIGEN_CONST_MATRIX_FROM_SIGNAL(support,(*contact.supportSIN)(t));
	    const int n0 = contact.range.first, n1 = contact.range.second,
	      r=6*contact.position, nbp = support.cols();
	    assert( ((n1-n0) % 3 == 0) && nbp == (n1-n0)/3);

	    for( int i=0;i<nbp;++i )
	      {
		assert( n0+3*(i+1)<=X.rows() && r+6<=X.cols() );
		X.block(n0+3*i,r, 3,3) = Matrix3d::Identity();
		Block<SigMatrixXd> px = X.block(n0+3*i,r+3, 3,3);
		sotSolverMotion::preCross(-support.col(i),px);
	      }
	  }

	return mlX;
      }

      // V
      ml::Matrix& SolverMotionReduced::
      freeMotionBaseSOUT_function( ml::Matrix& mlV,int t )
      {
	//std::cout << "Starting .V ... " << std::endl;
	using namespace Eigen;
	EIGEN_CONST_MATRIX_FROM_SIGNAL(B,inertiaSqrootInvSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(Jc,JcSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(X,forceGeneratorSOUT(t));
	const int & nq = sizeConfigurationSOUT(t);
	assert( Jc.cols()==nq && B.rows() == nq );
	
	//MatrixXd Gt = (Jc*B.triangularView<Eigen::Upper>()).transpose();  // If X is full-column rank
	MatrixXd Gt = (X*Jc*B.triangularView<Eigen::Upper>()).transpose();
	sotDEBUG(40) << "Gt = " << (soth::MATLAB)Gt << std::endl;
	//std::cout << "G: " << Gt.rows() << " x " << Gt.cols() << std::endl;
	Gt_qr.compute( Gt );
	Gt_qr.setThreshold(1e-3);
	G_rank = Gt_qr.rank();
	const unsigned int freeRank = nq-G_rank;
	sotDEBUG(40) << "Q = " << (soth::MATLAB)(MatrixXd)Gt_qr.householderQ()  << std::endl;

	/* Jc = [ L 0 ] Q' = [L 0] [ V_perp' ; V' ]
	   Jc' = Q [ R; 0 ] = [ V_perp V ] [ R; 0 ]
	   V = Q [ 0 ; I ].
	*/
	EIGEN_MATRIX_FROM_MATRIX(V,mlV,nq,freeRank);
	//std::cout << "time " << t << " - V created: " << nq << " x " << freeRank << std::endl;
	//assert( G_rank == Jc.rows() );
	//assert( G_rank == X.rows() );
	//std::cout << "G rank: " << G_rank << " - X rows: " << X.rows() << std::endl;
	assert( Gt_qr.householderQ().cols()==nq && Gt_qr.householderQ().rows()==nq );
	//std::cout << "Q cols: " << Gt_qr.householderQ().cols() << std::endl;
	//std::cout << "Q rows: " << Gt_qr.householderQ().rows() << std::endl;
	V.topRows(nq-freeRank).fill(0);
	V.bottomRows(freeRank) = MatrixXd::Identity(freeRank,freeRank);
	V.applyOnTheLeft( Gt_qr.householderQ() );
	return mlV;
      }

      // K
      ml::Matrix& SolverMotionReduced::
      freeForceBaseSOUT_function( ml::Matrix& mlK,int t )
      {
	using namespace Eigen;
	using soth::MATLAB;
	using std::endl;
	EIGEN_CONST_MATRIX_FROM_SIGNAL(X,forceGeneratorSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(Jc,JcSOUT(t));
	const int & nf = sizeForcePointSOUT(t);
	assert( X.rows()==nf );

	/* J[:,1:6] = [ R_1  X_1 ; ... ; R_N X_N ]
	 * with R_i the rotation of the frame where contact point i is expressed
	 * and X_i the skew of the vector from waist to contact point. */
	X_qr.compute( X*(Jc.leftCols(6)) );
	X_qr.setThreshold(1e-3);
	const unsigned int freeRank = nf-X_qr.rank();
	assert( X_qr.rank()==6 );

	sotDEBUG(5) << "JSb = " << (MATLAB)( (MatrixXd)((X*Jc).leftCols(6)) ) << endl;
	sotDEBUG(5) << "Q = " << (MATLAB)(MatrixXd)X_qr.householderQ()  << endl;

	EIGEN_MATRIX_FROM_MATRIX(K,mlK,nf,freeRank);
	K.topRows(X_qr.rank()).fill(0);
	K.bottomRows(freeRank) = MatrixXd::Identity(freeRank,freeRank);
	K.applyOnTheLeft( X_qr.householderQ() );

	return mlK;
      }

      void SolverMotionReduced::
      resizeSolver( void )
      {
	sotDEBUGIN(15);
	const int & npsi = sizeActuationSOUT.accessCopy(),
	  & nf = sizeForcePointSOUT.accessCopy(),
	  & nu = sizeMotionSOUT.accessCopy(),
	  & nq = sizeConfigurationSOUT.accessCopy(),
	  nbTasks = stack.size(),
	  nx = nf+nu;

	bool toBeResized = hsolver==NULL
	  || (nu+nf)!=(int)hsolver->sizeProblem
	  || stack.size()+3!= hsolver->nbStages();

	/* Resize the force-motion level. */
	assert( (nf%3)==0 );
	if( Cforce.rows()!=6 || Cforce.cols()!=nx || bforce.size()!=6)
	  {
	    Cforce.resize(6,nx);
	    bforce.resize(6);
	    toBeResized = true;
	  }

	/* Resize the normal forces inequality level. */
	if( Cfnormal.rows()!=nf/3 || Cfnormal.cols()!=nx || bfnormal.size()!=nf/3)
	  {
	    Cfnormal.resize(nf/3,nx);
	    bfnormal.resize(nf/3);
	    toBeResized = true;
	  }

	/* Resize the task levels. */
	if( Ctasks.size()!=nbTasks || btasks.size()!=nbTasks )
	  {
	    Ctasks.resize(nbTasks);
	    btasks.resize(nbTasks);
	  }
	for( int i=0;i<(int)stack.size();++i )
	  {
	    const int ntask = stack[i]->taskSOUT.accessCopy().size();
	    if( ntask != btasks[i].size()
		|| ntask != Ctasks[i].rows()
		|| nx != Ctasks[i].cols() )
	      {
		Ctasks[i].resize( ntask,nx );
		btasks[i].resize( ntask );
		toBeResized = true;
	      }
	  }

	/* Resize the final level. */
	if( Czero.cols()!=nx || Czero.rows()!=nq-6 || bzero.size()!=nq-6 )
	  {
	    Czero.resize(nq-6,nx);
	    bzero.resize(nq-6);
	    toBeResized = true;
	  }

	/* Rebuild the solver. */
	if( toBeResized )
	  {
	    sotDEBUG(1) << "Resize all." << std::endl;
	    hsolver = hcod_ptr_t(new soth::HCOD(nx,nbTasks+3));

	    hsolver->pushBackStage( Cforce,   bforce );
	    hsolver->stages.back()->name = "force";
	    hsolver->pushBackStage( Cfnormal, bfnormal);
	    hsolver->stages.back()->name = "fnormal";

	    for( int i=0;i<(int)stack.size();++i )
	      {
		hsolver->pushBackStage( Ctasks[i], btasks[i] );
		hsolver->stages.back()->name = stack[i]->getName();
	      }

	    hsolver->pushBackStage( Czero, bzero );
	    hsolver->stages.back()->name = "zero";

	    solution.resize( nx );
	  }
      }

      /* The drift is equal to: d = Gc^+ ( xcddot - Jcdot qdot ). */
      ml::Vector& SolverMotionReduced::
      driftContactSOUT_function( ml::Vector &mlres, int t )
      {
	/* BV has already been computed, but I don't know if it is the best
	   idea to go for it a second time. This suppose that the matrix has
	   not been modified in between. It should work, but start with that if
	   you are looking for a bug in ddq. */
	/* Same for Gt_qr. */
	using soth::MATLAB;
	using namespace sotSolverMotion;
	using namespace Eigen;

	const int nq = sizeConfigurationSOUT(t);
	JcSOUT(t); // To force the computation of forceDrift.
	freeMotionBaseSOUT(t); // To force the computation of G_svd.
	EIGEN_VECTOR_FROM_VECTOR(res,mlres,nq);
	sotDEBUG(40) << "fdrift = " << (MATLAB)forceDrift << std::endl;

	// const int nphi = sizeForceSpatialSOUT(t);
	// res.head(nphi) = forceDrift; res.tail( nq-nphi ).fill(0);

	const int nf = sizeForcePointSOUT(t);
	res.head(nf) = forceDrift; res.tail( nq-nf ).fill(0);
	//std::cout << "forceDrift in delta: " << std::endl << res << std::endl;

	Gt_qr.solveTransposeInPlace( res );
	sotDEBUG(40) << "drift = " << (MATLAB)res << std::endl;

	return mlres;
      }

      ml::Vector& SolverMotionReduced::
      solutionSOUT_function( ml::Vector &mlres, int t )
      {
	sotDEBUG(15) << " # In time = " << t << std::endl;
	using namespace soth;
	using namespace Eigen;
	using namespace sotSolverMotion;

	precomputeSOUT(t);
	EIGEN_CONST_MATRIX_FROM_SIGNAL( sB,   inertiaSqrootInvSIN(t));   // B
	EIGEN_CONST_MATRIX_FROM_SIGNAL( sBi,  inertiaSqrootSIN(t));      // B^{-1}
	EIGEN_CONST_MATRIX_FROM_SIGNAL( V,    freeMotionBaseSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( K,    freeForceBaseSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( Jc,   JcSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( Xc,   forceGeneratorSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( b,    dyndriftSIN(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( qdot, velocitySIN(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( drift,driftContactSOUT(t));
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> B(sB);
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> Bi(sBi);

	const int & nf = sizeForcePointSOUT(t), 
	  & nq = sizeConfigurationSOUT(t),
	  & nu = sizeMotionSOUT(t), nx=nf+nu;
	resizeSolver();

	assert( (nf%3)==0 );

	sotDEBUG(1) << "b = "  << (MATLAB)b   << std::endl;
	sotDEBUG(1) << "B = "  << (MATLAB)sB  << std::endl;
	sotDEBUG(1) << "Bi = " << (MATLAB)sBi << std::endl;
	sotDEBUG(1) << "V = "  << (MATLAB)V   << std::endl;
	sotDEBUG(1) << "K = "  << (MATLAB)K   << std::endl;
	sotDEBUG(1) << "Jc = " << (MATLAB)Jc  << std::endl;
	sotDEBUG(1) << "Xc = " << (MATLAB)Xc  << std::endl;

	if( dampingSIN ) //damp?
	  {
	    sotDEBUG(5) << "Using damping. " << std::endl;
	    hsolver->setDamping( 0 );
	    hsolver->useDamp( true );
	    hsolver->stages.back()->damping( dampingSIN(t) );
	  }
	else
	  {
	    sotDEBUG(5) << "Without damping. " << std::endl;
	    hsolver->useDamp( false );
	  }

	/* SOT reducing only the motion:
	 * 1.  S*B^{-T}*V*u + S*J'*f = -S*(b+B^{-T}*delta)
	 * 2.  fn <= 0
	 * 3.  Ji*B*V*u = ddei* - dJi*dq -Ji**B*delta
	 * 4.  Keep posture

	 * 1.  [ Sb*B^{-T}*V  Sb*J' ]  = -Sb*(b+B^{-T}*delta)
	 * 2.  [      0        Sf   ] <= 0
	 * 3.  [ Ji*B*V       0     ]  = ddei* - dJi*dq -Ji**B*delta
	 * 4.  Keep posture
	 */

#define COLS_U leftCols( nu )
	//#define COLS_F rightCols( npsi )
#define COLS_F rightCols( nf )
#define ROWS_FF topRows( 6 )
#define ROWS_ACT bottomRows( nq-6 )

	/* -1- : S*B^{-T}*V*u + S*J'*f = -S*(b+B^{-T}*delta) */
	{
	  MatrixXd SbBitV( 6,nu );
	  SbBitV = (sBi.transpose().topLeftCorner(6,6).triangularView<Lower>() * V.ROWS_FF);
	  sotDEBUG(15) << "SbBitV = " << (MATLAB)SbBitV << std::endl;

	  MatrixXd SbJt(6,nf);
	  SbJt = ( Xc*(Jc.leftCols(6)) ).transpose();
	  assert( SbJt.cols()==nbForce*3);

	  VectorXd SbBitDelta( 6 );
	  SbBitDelta
	    = -(b.ROWS_FF + sBi.transpose().topLeftCorner(6,6).triangularView<Lower>()*drift.ROWS_FF);
	  sotDEBUG(15) << "SbBitdelta = " << (MATLAB)SbBitDelta << std::endl;

	  Cforce.COLS_U = SbBitV;
	  Cforce.COLS_F = SbJt;
	  vbAssign( bforce, SbBitDelta );
	}
	sotDEBUG(15) << "Cforce = "     << (MATLAB)Cforce << std::endl;
	sotDEBUG(1) << "bforce = "     << bforce << std::endl;

	/* The matrix B*V has to be stored to avoid unecessary
	 * recomputation. */
	BV = B*V; // TODO: compute B*drift the same way.
	sotDEBUG(15) << "BV = "     << (MATLAB)BV << std::endl;
	sotDEBUG(15) << "B = "     << (MATLAB)(MatrixXd)B << std::endl;


	/* -2- : fnormal <= 0   */
	{
	  MatrixXd fnormal(nf/3,nf);
	  fnormal.setZero();
	  for( int i=0;i<nf/3;i++)
	    {
	      fnormal(i,(i+1)*3-1)=1;
	    }
	  Cfnormal.COLS_U.setZero();
	  Cfnormal.COLS_F = fnormal;
	  for( int i=0;i<nf/3;i++) bfnormal[i] = Bound(0, Bound::BOUND_SUP);
	}

	/* -3- : Ji*B*V*u = ddei* - dJi*dq -Ji**B*delta */
	for( int i=0;i<(int)stack.size();++i )
	  {
	    dyninv::TaskDynPD & task = * stack[i];
	    MatrixXd & Ctask1 = Ctasks[i];
	    VectorBound & btask1 = btasks[i];

	    EIGEN_CONST_MATRIX_FROM_SIGNAL(Jdot,task.JdotSOUT(t));
	    EIGEN_CONST_MATRIX_FROM_SIGNAL(J,task.jacobianSOUT(t));
	    const dg::sot::VectorMultiBound & ddx = task.taskSOUT(t);

	    sotDEBUG(5) << "ddx"<<i<<" = " << ddx << std::endl;
	    sotDEBUG(25) << "J"<<i<<" = " << (MATLAB)J << std::endl;
	    sotDEBUG(45) << "Jdot"<<i<<" = " << (MATLAB)Jdot << std::endl;
	    
	    assert( Ctask1.rows() == ddx.size() && btask1.size() == ddx.size() );
	    assert( J.rows()==ddx.size() && J.cols()==nq && (int)ddx.size()==ddx.size() );
	    assert( Jdot.rows()==ddx.size() && Jdot.cols()==nq );

	    Ctask1.COLS_U = J*BV;	    Ctask1.COLS_F.fill(0);
	    VectorXd Jdqd = Jdot*qdot;
	    vbAssign(btask1,ddx);
	    vbSubstract(btask1,Jdqd);
	    sotDEBUG(45) << "Jdqd"<<i<<" = " << (MATLAB)Jdqd << std::endl;
	    sotDEBUG(45) << "JBdc"<<i<<" = " << (MATLAB)(MatrixXd)(J*B*drift) << std::endl;
	    vbSubstract(btask1,(VectorXd)(J*B*drift));

	    /* TODO: account for the contact drift. */

	    sotDEBUG(45) << "Ctask"<<i<<" = " << (MATLAB)Ctask1 << std::endl;
	    sotDEBUG(45) << "btask"<<i<<" = " << btask1 << std::endl;
	  }

	/* -4- */
	/* Czero = [ BV 0 ] */
	assert( Czero.cols() == nx && Czero.rows()==nq-6 && bzero.size() ==nq-6 );
	assert( nbDofs+6 == nq );
	Czero.COLS_U = BV.ROWS_ACT;
	Czero.COLS_F.setZero();
	const double & Kv = breakFactorSIN(t);
	EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,velocitySIN(t));
	if( postureSIN && positionSIN )
	   {
	     EIGEN_CONST_VECTOR_FROM_SIGNAL(qref,postureSIN(t));
	     EIGEN_CONST_VECTOR_FROM_SIGNAL(q,positionSIN(t));
	     const double Kp = .25*Kv*Kv;
	     vbAssign( bzero,(-Kp*(q-qref)-Kv*dq).tail(nbDofs) );
	   }
	else
	  {	vbAssign(bzero,(-Kv*dq).tail(nbDofs));	  }
	/* TODO: account for the contact drift. */
	vbSubstract(bzero,((VectorXd)(B*drift)).ROWS_ACT  );
	sotDEBUG(15) << "Czero = "     << (MATLAB)Czero << std::endl;
	sotDEBUG(1) << "bzero = "     << bzero << std::endl;

	/* Run solver */
	sotDEBUG(1) << "Initial config." << std::endl;
	hsolver->reset();
	hsolver->setInitialActiveSet();
	sotDEBUG(1) << "Run for a solution." << std::endl;
	hsolver->activeSearch(solution);
	sotDEBUG(1) << "solution = " << (MATLAB)solution << std::endl;
	EIGEN_VECTOR_FROM_VECTOR(res,mlres,nx);
	res=solution;

	// Small verif:
	if( Ctasks.size()>0 )
	  {
	    sotDEBUG(1) << "ddx0 = " << (MATLAB)(VectorXd)(Ctasks[0]*solution) << std::endl;
	    sotDEBUG(1) << "ddx0 = " << btasks[0] << std::endl;
	  }


	// To Debug
	// for( int i=0;i<(int)stack.size();++i )
	//   {
	//     MatrixXd & Ctask1 = Ctasks[i];
	//     VectorBound & btask1 = btasks[i];
	//     std::cout << "Ctask"<<i<<" = " << (MATLAB)(Ctask1*solution) << std::endl;
	//     std::cout << "btask"<<i<<" = " << btask1 << std::endl << std::endl;
	//   }

	sotDEBUGOUT(15);
	return mlres;
      }

      ml::Vector& SolverMotionReduced::
      reducedControlSOUT_function( ml::Vector& res,int t )
      {
	EIGEN_CONST_VECTOR_FROM_SIGNAL(x,solutionSOUT(t));
	const int & nu = sizeMotionSOUT(t);
	EIGEN_VECTOR_FROM_VECTOR(u,res,nu);
	u = x.head(nu);

	return res;
      }

      // Changed to display the "punctual forces"
      ml::Vector& SolverMotionReduced::
      forcesSOUT_function( ml::Vector& res,int t )
      {
	EIGEN_CONST_VECTOR_FROM_SIGNAL(x,solutionSOUT(t));
	const int & nf = sizeForcePointSOUT.accessCopy();
	EIGEN_VECTOR_FROM_VECTOR(forcesPoint,res,nf);
	forcesPoint = x.tail(nf);

	return res;
      }

      ml::Vector& SolverMotionReduced::
      accelerationSOUT_function( ml::Vector& mlddq,int t )
      {
	const int & nq = sizeConfigurationSOUT(t);
	EIGEN_CONST_VECTOR_FROM_SIGNAL(u,reducedControlSOUT(t));
	EIGEN_VECTOR_FROM_VECTOR(ddq,mlddq,nq);
	/* BV has already been computed, but I don't know if it is the best
	 * idea to go for it a second time. This suppose that the matrix has
	 * not been modified in between. It should work, but start with that if
	 * you are looking for a bug in ddq. */

	using soth::MATLAB;
	using namespace sotSolverMotion;
	using namespace Eigen;

	EIGEN_CONST_VECTOR_FROM_SIGNAL( drift, driftContactSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( sB,    inertiaSqrootInvSIN(t));
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> B(sB);
	sotDEBUG(40) << "drift = " << (MATLAB)drift << std::endl;
	sotDEBUG(40) << "BV = "    << (MATLAB)BV    << std::endl;
	sotDEBUG(40) << "B = "     << (MATLAB)sB    << std::endl;
	sotDEBUG(40) << "u = "     << (MATLAB)u     << std::endl;
	ddq = BV*u + B*drift;

	/* --- verif --- */
	{
	  EIGEN_CONST_MATRIX_FROM_SIGNAL(Jc,JcSOUT(t));
	  sotDEBUG(40) << "fdrift = " << (MATLAB)forceDrift                     << std::endl;
	  sotDEBUG(40) << "Jqdd = "   << (MATLAB)(MatrixXd)(Jc*ddq)             << std::endl;
	  sotDEBUG(40) << "diff = "   << (MATLAB)(MatrixXd)(Jc*ddq-forceDrift)  << std::endl;
	  sotDEBUG(40) << "ndiff = "  << (Jc*ddq-forceDrift).norm()             << std::endl;
	}

	return mlddq;
      }

      // COMPUTE PSI (Is the inversion of K correct????)
      ml::Vector& SolverMotionReduced::
      reducedForceSOUT_function( ml::Vector& res,int t )
      {
	using namespace Eigen;
	using namespace soth;
	EIGEN_CONST_MATRIX_FROM_SIGNAL(sB,inertiaSqrootInvSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(sBi,inertiaSqrootSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(V,freeMotionBaseSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(K,freeForceBaseSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(Jc,JcSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(Xc,forceGeneratorSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL(b,dyndriftSIN(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL(drift,driftContactSOUT(t));
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> B(sB);
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> Bi(sBi);
	EIGEN_CONST_VECTOR_FROM_SIGNAL(u,reducedControlSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL(f,forcesSOUT(t));  // f, instead of psi

	const int & npsi = sizeActuationSOUT(t),
	  & nf = sizeForcePointSOUT(t);
	//EIGEN_VECTOR_FROM_VECTOR(psi,res,nf);
	EIGEN_VECTOR_FROM_VECTOR(psi,res,npsi);

	VectorXd f1(nf);
	// = Sb( B^-T( Vu+delta ) + b )
	f1.ROWS_FF	
	  = sBi.transpose().topLeftCorner(6,6).triangularView<Lower>()
	  * ( V.ROWS_FF*u + drift.ROWS_FF ) + b.ROWS_FF;
	f1.tail( nf-6 ).setZero();
	//sotDEBUG(15) << "SBVu = "     << (MATLAB)f1 << std::endl;

	X_qr.solveTransposeInPlace(f1);
	//sotDEBUG(15) << "SJptSBVu = "     << (MATLAB)f1 << std::endl;

	f1 += f;
	//sotDEBUG(15) << "f = "     << (MATLAB)f1 << std::endl;

	psi = K.colPivHouseholderQr().solve(f1);

	// /* phi = X' f */
	// VectorXd phi = Xc.transpose()*f;
	// BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	//   {
	//     Contact& contact = pContact.second;
	//     const int r = 6*contact.position;
	//     ml::Vector mlphii;
	//     EIGEN_VECTOR_FROM_VECTOR( phii,mlphii,6 );
	//     assert( r+6<=phi.size() );

	//     phii = phi.segment(r,6);
	//     (*contact.forceSOUT) = mlphii;
	//   }

	return res;
      }

      ml::Vector& SolverMotionReduced::
      forcesNormalSOUT_function( ml::Vector& res,int t )
      {
	using namespace Eigen;
	using namespace soth;
	EIGEN_CONST_VECTOR_FROM_SIGNAL(solution,solutionSOUT(t));
	const int & nfn = Cfnormal.rows();
	EIGEN_VECTOR_FROM_VECTOR(fn,res,nfn);
	fn = Cfnormal*solution;

	for( int i=0;i<nfn;++i )
	  {
	    fn[i] -= bfnormal[i].getBound( bforce[i].getType() );
	  }

	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int nf0 = contact.range.first / 3;
	    const int nff = contact.range.second / 3;
	    assert( (contact.range.first%3) == 0);
	    assert( (contact.range.second%3) == 0);
	    assert( nff<=fn.size() );

	    ml::Vector mlfni;
	    EIGEN_VECTOR_FROM_VECTOR( fni,mlfni,nff-nf0 );
	    fni = fn.segment(nf0,nff-nf0);
	    (*contact.fnSOUT) = mlfni;
	  }

	return res;
      }

      ml::Vector& SolverMotionReduced::
      activeForcesSOUT_function( ml::Vector& res,int t )
      {
	using namespace Eigen;
	using namespace soth;
	EIGEN_CONST_VECTOR_FROM_SIGNAL(solution,solutionSOUT(t));
	Stage & stf = *hsolver->stages.front();
	EIGEN_VECTOR_FROM_VECTOR(a,res,stf.sizeA());

	VectorXd atmp(stf.nbConstraints()); atmp=stf.eactive(atmp);

	return res;
      }

      ml::Vector& SolverMotionReduced::
      torqueSOUT_function( ml::Vector& mltau, int t)
      {
	const int & nq = sizeConfigurationSOUT(t);
	EIGEN_VECTOR_FROM_VECTOR(tau,mltau,nq-6);
	
	using namespace Eigen;
	EIGEN_CONST_MATRIX_FROM_SIGNAL( sBi,   inertiaSqrootSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( V,     freeMotionBaseSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( Jc,    JcSOUT(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL( X,     forceGeneratorSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( b,     dyndriftSIN(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( drift, driftContactSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( u,     reducedControlSOUT(t));
	EIGEN_CONST_VECTOR_FROM_SIGNAL( f,     forcesSOUT(t));
	Eigen::TriangularView<const_SigMatrixXd,Eigen::Upper> Bi(sBi);

	// tau = S( B^{-T}*( Vu+delta ) + b)
	tau = sBi.transpose().ROWS_ACT * ( V*u + drift )
	  + b.ROWS_ACT 
	  + (Jc.transpose()).ROWS_ACT * X.transpose()*f ;

	/* --- verif --- */
	{
	  using soth::MATLAB;
	  sotDEBUG(40) << "torque = " << (MATLAB)tau   << std::endl;
	}

	return mltau;
      }

      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      void SolverMotionReduced::
      display( std::ostream& os ) const
      {
	os << "SolverMotionReduced "<<getName() << ": " << nbDofs <<" joints." << std::endl;
	try{
	  os <<"solution = "<< solutionSOUT.accessCopy() <<std::endl << std::endl;
	}  catch (dynamicgraph::ExceptionSignal e) {}
	//stack_t::display(os);
	dispContacts(os);
      }

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph

