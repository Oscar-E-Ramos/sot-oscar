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

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>

#include <sot-oscar/velocity-control.h>
#include <sot-dyninv/commands-helper.h>

#include <boost/foreach.hpp>
#include <Eigen/Dense>


namespace dynamicgraph
{
  namespace sot
  {
    namespace oscar
    {

      namespace dg = ::dynamicgraph;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VelocityControl,"VelocityControl");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      VelocityControl::
      VelocityControl( const std::string & name )
	: Entity(name)
	, mlpositionPrevious()
	, error()
	, errorSum()
	,CONSTRUCT_SIGNAL_IN( refVelocity, ml::Vector)
	,CONSTRUCT_SIGNAL_IN( position,  ml::Vector)
	,CONSTRUCT_SIGNAL_IN( dt, double)
	,CONSTRUCT_SIGNAL_IN( kp, double)
	,CONSTRUCT_SIGNAL_IN( ki, double)
      
	,CONSTRUCT_SIGNAL_OUT(velocity, ml::Vector, 
			      dtSIN << positionSIN)
	,CONSTRUCT_SIGNAL_OUT(control, ml::Vector, 
			      velocitySOUT << refVelocitySIN << kpSIN << kiSIN)
      {
	signalRegistration( refVelocitySIN << positionSIN << dtSIN << kpSIN
			    << kiSIN << velocitySOUT << controlSOUT );
	
	std::cout << "<velocityControl> 1" << std::endl;

      }


      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /*
	Computation of the measured velocity based on the measured position, by finite
	differences
      */
      ml::Vector& VelocityControl::
      velocitySOUT_function( ml::Vector& res, int time )
      {
      	const ml::Vector & mlposition = positionSIN(time);
	const double & dt = dtSIN(time);

	res.resize( mlposition.size() );

	// Initialization of the previous position (to initial position)
	if(mlpositionPrevious.size() == 0){
	  mlpositionPrevious.resize( mlposition.size() );
	  for( unsigned int i=0; i<mlposition.size(); i++)
	    mlpositionPrevious(i) = mlposition(i);
	}

	// Calculation by finite differences
	for( unsigned int i=0; i<mlposition.size(); i++)
	    res(i) = (mlposition(i) - mlpositionPrevious(i))/dt;

	// Storage of the previous position
	for( unsigned int i=0; i<mlposition.size(); i++)
	  mlpositionPrevious(i) = mlposition(i);

	// Debug:
	// std::cout << "mlrefVelocity: (";
	// for( unsigned int i=0; i<mlrefVelocity.size(); i++)
	//   std::cout << "mlrefVelocity" << 

	return res;
      }

      /* 
	 Computation of the KI control law to close the loop at the velocity level
      */
      ml::Vector& VelocityControl::
      controlSOUT_function( ml::Vector& res, int time)
      {
	const ml::Vector &mlrefVelocity = refVelocitySIN(time);
	const ml::Vector &mlmeasuredVelocity = velocitySOUT(time);

	const double &kp = kpSIN(time);
	const double &ki = kiSIN(time);

	res.resize( mlrefVelocity.size() );
	error.resize( mlrefVelocity.size() );

	// Initialize the integral to zero
	if( errorSum.size()==0 ){
	  errorSum.resize( mlrefVelocity.size());
	  for( unsigned int i=0; i<errorSum.size(); i++)
	    errorSum(i) = 0;
	}
	
	// Leave the first 6 elements (free-flyer) as they are
	for( unsigned int i=0; i<6; i++)
	  res(i) = mlrefVelocity(i);

	// Compute the error, the integral and the control law for the actuated joints
	for( unsigned int i=6; i<mlrefVelocity.size(); i++){
	  error(i) = mlrefVelocity(i) - mlmeasuredVelocity(i);
	  errorSum(i) += error(i);
	  res(i) = kp*error(i) + ki*errorSum(i);
	  //res(i) = mlrefVelocity(i);
	}

	return res;
      }
      

      /* ---------------------------------------------------------------------- */
      /* --- HELPERS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      
      
      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      void VelocityControl::
      display( std::ostream& os ) const
      {
      	os << "VelocityControl " << getName() << ": " << std::endl;
      	//os << "--- LIST ---  " << std::endl;
      	// BOOST_FOREACH( dg::sot::FeatureAbstract* feature,featureList )
      	//   {
      	//     os << "-> " << feature->getName() << std::endl;
      	//   }
      }

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph

