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

#include <sot-oscar/velocity-correction.h>
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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VelocityCorrection,"VelocityCorrection");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      VelocityCorrection::
      VelocityCorrection( const std::string & name )
	: Entity(name)

	,CONSTRUCT_SIGNAL_IN(velocityIN,  ml::Vector)
	,CONSTRUCT_SIGNAL_OUT(velocityOUT, ml::Vector, 
			     velocityINSIN)
      {
	signalRegistration( velocityINSIN << velocityOUTSOUT );

      }


      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /*
	Computation of the output velocity based on the input velocity and the
       experimentally determined control rule
      */
      ml::Vector& VelocityCorrection::
      velocityOUTSOUT_function( ml::Vector& res, int time )
      {
      	sotDEBUGIN(15);
	
      	const ml::Vector & mlvelocity = velocityINSIN(time);
	res.resize( mlvelocity.size() );

	for( unsigned int i=0; i<mlvelocity.size(); i++)
	  res(i) = applyVelocityTransferFunction( mlvelocity, i); 

	return res;
      }


      /* ---------------------------------------------------------------------- */
      /* --- HELPERS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      double VelocityCorrection::
      applyVelocityTransferFunction( const ml::Vector & mlvelocity, 
				     unsigned int i)
      {
	double vel;
	bool modify = false;

	/* Variables to store the parameters of the function */
	double paramPos[4];
	double paramNeg[4];

	
	/* Define the transformation rule for each joint */
	if(i==(23+6)) {
	  paramPos[0] = -20.401;  paramPos[1] = 113.97;
	  paramPos[2] = -12.238;  paramPos[3] = 1.8882;
	  paramNeg[0] = 52.377;   paramNeg[1] = 22.157;
	  paramNeg[2] = -0.4941;  paramNeg[3] = 0.25563;
	  modify = true;
	}
	else if(i==(24+6)) {
	  paramPos[0] = 47.662;   paramPos[1] = 63.097;
	  paramPos[2] = -4.6131;  paramPos[3] = 0.77461;
	  paramNeg[0] = 91.932;   paramNeg[1] = 43.167;
	  paramNeg[2] = 1.5825;   paramNeg[3] = 0.63106;
	  modify = true;
	}
	else if(i==(25+6)) {
	  paramPos[0] = -18918;   paramPos[1] = 28131;
	  paramPos[2] = -8171.9;  paramPos[3] = 1707.5;
	  paramNeg[0] = -11562;   paramNeg[1] = 17333;
	  paramNeg[2] = -4297.9;  paramNeg[3] = 759.98;
	  modify = true;
	}
	else if(i==(26+6)) {
	  paramPos[0] = 53.627;   paramPos[1] = 26.974;
	  paramPos[2] = -0.92833; paramPos[3] = 0.19597;
	  paramNeg[0] = 52.901;   paramNeg[1] = 31.942;
	  paramNeg[2] = -1.819;   paramNeg[3] = 0.22741;
	  modify = true;
	}

	/* Modify the joint velocity according to the 4-th order root, if the parameters 
	   (th0, th1, th2, th3) have been specified */
	if (modify){
	  if(mlvelocity(i) >= 0)
	    /* f(u) = (th0*u^4 + th1*u^3 + th2*u^2 + th3*u)^(1/4)   */
	    vel = pow( paramPos[0]*pow(mlvelocity(i),4) + paramPos[1]*pow(mlvelocity(i),3) +
		       paramPos[2]*pow(mlvelocity(i),2) + paramPos[3]*mlvelocity(i), 0.25 );
	  else
	    /* f(u) = -(th0*u^4 - th1*u^3 + th2*u^2 - th3*u)^(1/4)   */
	    vel = -pow( paramNeg[0]*pow(mlvelocity(i),4) - paramNeg[1]*pow(mlvelocity(i),3) +
		        paramNeg[2]*pow(mlvelocity(i),2) - paramNeg[3]*mlvelocity(i), 0.25 );
	  
	  // std::cout << "For joint " << i-6 << ": ";
	  // std::cout << "invel = " << double(mlvelocity(i)) << ", outvel = " << double(vel);
	  // std::cout << std::endl;

	}
	else
	  vel = mlvelocity(i);
	
	return vel;
      } 
      
      
      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      void VelocityCorrection::
      display( std::ostream& os ) const
      {
      	os << "VelocityCorrection " << getName() << ": " << std::endl;
      	//os << "--- LIST ---  " << std::endl;
      	// BOOST_FOREACH( dg::sot::FeatureAbstract* feature,featureList )
      	//   {
      	//     os << "-> " << feature->getName() << std::endl;
      	//   }
      }

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph

