/*
 * Copyright 2013, Oscar Efrain Ramos Ponce, LAAS-CNRS
 *
 * This file is part of sot-oscar.
 * sot-oscar is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * sot-oscar is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-oscar.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot-dyninv/commands-helper.h>
#include <sot-oscar/foot-collision-grid.h>

#include <sot-dyninv/signal-helper.h>
#include <soth/HCOD.hpp>
#include <sot-dyninv/mal-to-eigen.h>

#include <vector>

// #define MYDEBUG 1
// #define OUTPUT_POINTS 1

/* For output of debug strings */
#ifdef MYDEBUG
#define DEBUG_MSG(str) do { std::cout << "<DEB> " << str; } while(false)
#else
#define DEBUG_MSG(str) do {} while(false)
#endif

/* For the output of the points */
#ifdef OUTPUT_POINTS
#define OUTPUT_PTS(str) do { std::cout << str; } while(false)
#else
#define OUTPUT_PTS(str) do {} while(false)
#endif


namespace dynamicgraph
{
  namespace sot
  {
    namespace oscar
    {

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FootCollisionGrid,"FootCollisionGrid");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      FootCollisionGrid::
      FootCollisionGrid( const std::string & name )
	: Entity(name)
	,gridStepSize(0.01)
	,cornersWrtAnkle()
	,originWrtAnkle()
	,centerWrtAnkle()
	,CONSTRUCT_SIGNAL_IN(contactPointsIN, ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(footTransf, ml::Matrix)
	,CONSTRUCT_SIGNAL_OUT(gridStepSize, ml::double, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(footCornersWrtAnkle, ml::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(footCornersWrtWorld, ml::Matrix, 
			      footTransfSIN )
	,CONSTRUCT_SIGNAL_OUT(contactPointsFull, ml::Matrix, 
			      footTransfSIN << contactPointsINSIN )
	,CONSTRUCT_SIGNAL_OUT(contactPoints, ml::Matrix, 
			      footTransfSIN << contactPointsINSIN )	  
      {
	signalRegistration(contactPointsINSIN << footTransfSIN << gridStepSizeSOUT 
			   << contactPointsFullSOUT << contactPointsSOUT
			   << footCornersWrtAnkleSOUT << footCornersWrtWorldSOUT );
	footCornersWrtAnkleSOUT.setNeedUpdateFromAllChildren( true );
	gridStepSizeSOUT.setNeedUpdateFromAllChildren( true );
	initCommands();
      }

      void FootCollisionGrid::
      initCommands( void )
      {
	using namespace dynamicgraph::command;

	addCommand("setGridStepSize",
		   makeCommandVoid1(*this, &FootCollisionGrid::setGridStepSize,
				    dyninv::docCommandVoid1("Set the step size for the grid.",
							    "gridStepSize (double)")));

	addCommand("setFootCorners",
		   makeCommandVoid1(*this, &FootCollisionGrid::setFootCorners,
				    dyninv::docCommandVoid1("Set the corners of the foot.",
							    "footCorners (matrix)")));
	
      }
      
      /* ---------------------------------------------------------------------- */
      /* --- COMMANDS --------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /** Sets the step size for the grid that will be localized on the sole of the foot
	  \param [in] stepSize The step size of the grid
      */
      void FootCollisionGrid::
      setGridStepSize( const double & stepSize )
      {
	gridStepSize = stepSize;
      }

      /** Sets the coordinates of the foot corners with respect to the ankle frame. 
	  It assumes an initial position with equal z (horizontal plane) for all the points. The points 
	  are ordered starting from the one with minimum x, y components and going counter-clockwise. 
	  Also, the center of the foot is calculated and the 'grid initializer' function is called.
	  \param [in] footCorners Coordinates of the 4 foot corners (3x4), where each column is a point
      */
      void FootCollisionGrid::
      setFootCorners( const ml::Matrix & footCorners )
      {
	/* Corners: each column is a point (in R^3) */
	cornersWrtAnkle.resize(3,4);
	assert((footCorners.nbRows()==3) && (footCorners.nbCols()==4));
	
	EIGEN_CONST_MATRIX_FROM_SIGNAL(eigFootCorners, footCorners);
	
	/* Obtain the min indices for x and y (to be used to order the points */
	unsigned int ixmin1, ixmin2, iymin1, iymin2;
	calculateTwoMins(eigFootCorners, 0, ixmin1, ixmin2);   // Get min indices for x
	calculateTwoMins(eigFootCorners, 1, iymin1, iymin2);   // Get min indices for y

	/* Determine the indices for the points */
	unsigned int indP[4];
	if(ixmin1==iymin1)
	  { indP[0]=ixmin1; indP[1] = iymin2; indP[3] = ixmin2; }
	else if(ixmin1==iymin2)
	  { indP[0]=ixmin1; indP[1] = iymin1; indP[3] = ixmin2; }
	else if(ixmin2==iymin1)
	  { indP[0]=ixmin2; indP[1] = iymin2; indP[3] = ixmin1; }
	else if(ixmin2==iymin2)
	  { indP[0]=ixmin2; indP[1] = iymin1; indP[3] = ixmin1; }
	else
	  std::cerr << "No initial corner was found. Verify the feet corner values" << std::cout;

	/* Find the index max-max (for P2) */
	for(unsigned int i=0; i<4; i++){
	  if (indP[0]==i || indP[1]==i || indP[3]==i)
	    continue;
	  else{
	    indP[2]=i;
	    break;
	  }
	}

	/* Assign the ordered values to the corners */
	cornersWrtAnkle.col(0) = eigFootCorners.col(indP[0]);
	cornersWrtAnkle.col(1) = eigFootCorners.col(indP[1]);
	cornersWrtAnkle.col(2) = eigFootCorners.col(indP[2]);
	cornersWrtAnkle.col(3) = eigFootCorners.col(indP[3]);
	//cornersWrtAnkle = eigFootCorners;

	/* Calculate the center of the points */
	centerWrtAnkle.head(3) = 0.25*(cornersWrtAnkle.col(0) + cornersWrtAnkle.col(1) +
				      cornersWrtAnkle.col(2) + cornersWrtAnkle.col(3));
	centerWrtAnkle(3) = 1.0;
	DEBUG_MSG("CenterWrtAnkle: " << centerWrtAnkle << std::endl);
	  
	initializeGridWrtAnkle();

      }



      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      double& FootCollisionGrid::
      gridStepSizeSOUT_function( double& stepSize, int t)
      {
      	stepSize = gridStepSize;
      	return stepSize;
      }

      ml::Matrix& FootCollisionGrid::
      footCornersWrtAnkleSOUT_function( ml::Matrix& mlcorners, int t )
      {
	assert((cornersWrtAnkle.rows()==3) && (cornersWrtAnkle.cols()==4));
	mlcorners.resize(cornersWrtAnkle.rows(), cornersWrtAnkle.cols());

	for(int i=0; i<cornersWrtAnkle.cols(); i++){
	  mlcorners(0,i) = cornersWrtAnkle(0,i);
	  mlcorners(1,i) = cornersWrtAnkle(1,i);
	  mlcorners(2,i) = cornersWrtAnkle(2,i);
	}
	  
      	return mlcorners;
      }


      /** By default only 4 corners for the foot are allowed
       */
      ml::Matrix& FootCollisionGrid::
      footCornersWrtWorldSOUT_function( ml::Matrix& mlcorners, int t )
      {
	EIGEN_CONST_MATRIX_FROM_SIGNAL(TransfAnkleWrtWorld, footTransfSIN(t));
	
	assert((cornersWrtAnkle.rows()==3) && (cornersWrtAnkle.cols()==4));
	mlcorners.resize(cornersWrtAnkle.rows(), cornersWrtAnkle.cols());

	Eigen::Vector4d cornerWrtWorld(1,1,1,1);
	for(int i=0; i<cornersWrtAnkle.cols(); i++){
	  cornerWrtWorld.head(3) = cornersWrtAnkle.col(i);        // In homogeneous coordinates
	  cornerWrtWorld = TransfAnkleWrtWorld * cornerWrtWorld;  // In homogeneous coordinates
	  mlcorners(0,i) = cornerWrtWorld(0);
	  mlcorners(1,i) = cornerWrtWorld(1);
	  mlcorners(2,i) = cornerWrtWorld(2);
	}
      	return mlcorners;
      }


      ml::Matrix& FootCollisionGrid::
      contactPointsFullSOUT_function( ml::Matrix& mlpoints, int t )
      {
	EIGEN_CONST_MATRIX_FROM_SIGNAL(pointsIN, contactPointsINSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(TransfAnkleWrtWorld, footTransfSIN(t));
	gridContactPoints.clear();

	Eigen::Vector4d contactPoint; contactPoint(3) = 1.0;
	std::pair<int, int> indexGrid;
	int ncontactInGrid = 0;

	/* Get the current virtual origins of the foot in the world frame */
	Eigen::Vector4d originWrtWorld = TransfAnkleWrtWorld * originWrtAnkle;
	DEBUG_MSG("OriginWrtWorld = [" << originWrtWorld << "];" << std::endl);

	/* Vectors to be used for the increments in x and y */
	Eigen::Vector4d vx(1.0, 0.0, 0.0, 0.0);
	Eigen::Vector4d vy(0.0, 1.0, 0.0, 0.0);
	vx = TransfAnkleWrtWorld * vx;
	vy = TransfAnkleWrtWorld * vy;
	vx.normalize(); vy.normalize();
	DEBUG_MSG("vx=[" << vx.transpose() << "]; vy=[" << vy.transpose() << "];" << std::endl);
	
	/* Fill with '1' the grid elements where collision was detected */
	for(int i=0; i<pointsIN.rows(); i++) {
	  contactPoint.head(3) = pointsIN.row(i);   // In homogeneous coordinates
	  indexGrid.first  = int(floor( vx.dot(contactPoint-originWrtWorld) / gridStepSize ));
	  indexGrid.second = int(floor( vy.dot(contactPoint-originWrtWorld) / gridStepSize ));
	  DEBUG_MSG("Point" << i << " - Grid indices (x0, y0) = (" << indexGrid.first << ", "
		    << indexGrid.second << ")" << std::endl); 
	  if(grid[indexGrid]==0){
	    /* Update the grid value */
	    DEBUG_MSG("UPDATED GRID WITH: " << indexGrid.first << "  " << indexGrid.second << std::endl);
	    grid[indexGrid]=1;
	    ncontactInGrid ++;
	    
	    /* Get the point */
	    contactPoint(0) = originWrtAnkle(0) + 0.5*gridStepSize*(2*indexGrid.first+1);
	    contactPoint(1) = originWrtAnkle(1) + 0.5*gridStepSize*(2*indexGrid.second+1);
	    contactPoint(2) = originWrtAnkle(2);
	    contactPoint = TransfAnkleWrtWorld * contactPoint;
	    gridContactPoints.push_back(contactPoint);
	  }
	}
	
	/* Get the middle points of the grids with collision */
	mlpoints.resize(ncontactInGrid, 3);
	for(unsigned int i=0; i<gridContactPoints.size(); i++){
	  mlpoints(i,0) = gridContactPoints[i](0);
	  mlpoints(i,1) = gridContactPoints[i](1);
	  mlpoints(i,2) = gridContactPoints[i](2);
	}

	/* Reinitialize the grid to 0 */
	for (std::map<std::pair<int,int>, int>::iterator it=grid.begin(); it!=grid.end(); ++it)
	  it->second = 0;

	return mlpoints;
      }

      ml::Matrix& FootCollisionGrid::
      contactPointsSOUT_function( ml::Matrix& mlpoints, int t )
      {

	EIGEN_CONST_MATRIX_FROM_SIGNAL(pointsIN, contactPointsINSIN(t));
	EIGEN_CONST_MATRIX_FROM_SIGNAL(TransfAnkleWrtWorld, footTransfSIN(t));
	gridContactPoints.clear();
	cgalResult.clear();
	cgalPoints.clear();

	Eigen::Vector4d contactPoint; contactPoint(3) = 1.0;
	std::pair<int, int> indexGrid;
	int ncontactInGrid = 0;

	/* Get the current virtual origins of the foot in the world frame */
	Eigen::Vector4d originWrtWorld = TransfAnkleWrtWorld * originWrtAnkle;
	DEBUG_MSG("OriginWrtWorld = [" << originWrtWorld << "];" << std::endl);

	/* Vectors to be used for the increments in x and y */
	Eigen::Vector4d vx(1.0, 0.0, 0.0, 0.0);
	Eigen::Vector4d vy(0.0, 1.0, 0.0, 0.0);
	vx = TransfAnkleWrtWorld * vx;  vx.normalize();
	vy = TransfAnkleWrtWorld * vy;  vy.normalize();
	DEBUG_MSG("vx=[" << vx.transpose() << "]; vy=[" << vy.transpose() << "];" << std::endl);
	
	/* Fill with '1' the grid elements where collision was detected */
	for(int i=0; i<pointsIN.rows(); i++) {
	  
	  contactPoint.head(3) = pointsIN.row(i);   // In homogeneous coordinates
	  indexGrid.first  = int(floor( vx.dot(contactPoint-originWrtWorld) / gridStepSize ));
	  indexGrid.second = int(floor( vy.dot(contactPoint-originWrtWorld) / gridStepSize ));
	  DEBUG_MSG("Point" << i << " - Grid indices (x0, y0) = (" << indexGrid.first << ", "
		    << indexGrid.second << ")" << std::endl); 
	  
	  /* If the grid element has detected collision for the first time */
	  if(grid[indexGrid]==0){
	    DEBUG_MSG("UPDATED GRID WITH: " << indexGrid.first << "  " << indexGrid.second << std::endl);
	    /* Update the grid value */
	    grid[indexGrid]=1;
	    ncontactInGrid ++;
	    /* Compute the 2D convex hull */
	    cgalPoints.push_back(Point_2(indexGrid.first, indexGrid.second));
	  }
	}

	/* Get the convex hull in 2D using the grid indices */
	CGAL::convex_hull_2( cgalPoints.begin(), cgalPoints.end(), std::back_inserter(cgalResult) );
	DEBUG_MSG(cgalResult.size() << " points on the convex hull" << std::endl);

	int ix, iy;
	for(unsigned int i=0; i<cgalResult.size(); i++){
	  ix = int(cgalResult[i][0]);
	  iy = int(cgalResult[i][1]);
	  contactPoint(0) = originWrtAnkle(0) + 0.5*gridStepSize*(2*ix+1);
	  contactPoint(1) = originWrtAnkle(1) + 0.5*gridStepSize*(2*iy+1);
	  contactPoint(2) = originWrtAnkle(2);
	  contactPoint = TransfAnkleWrtWorld * contactPoint;
	  gridContactPoints.push_back(contactPoint);
	}
	
	/* Get the middle points of the grids with collision */
	mlpoints.resize(gridContactPoints.size(), 3);
	for(unsigned int i=0; i<gridContactPoints.size(); i++){
	  mlpoints(i,0) = gridContactPoints[i](0);
	  mlpoints(i,1) = gridContactPoints[i](1);
	  mlpoints(i,2) = gridContactPoints[i](2);
	}

	/* Reinitialize the grid to 0 */
	for (std::map<std::pair<int,int>, int>::iterator it=grid.begin(); it!=grid.end(); ++it)
	  it->second = 0;
	
	return mlpoints;
      }


      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /** Entity inheritance: gets the name */
      void FootCollisionGrid::
      display( std::ostream& os ) const
      {
      	os << "FootCollisionGrid "<< getName() << "." << std::endl;
      }

      /** Entity inheritance */
      void FootCollisionGrid::
      commandLine( const std::string& cmdLine,
      		   std::istringstream& cmdArgs,
      		   std::ostream& os )
      {
      	if( cmdLine == "help" )
      	  {
      	    os << "FootCollisionGrid:\n"
      	       << "\t- ." << std::endl;
      	    Entity::commandLine( cmdLine,cmdArgs,os );
      	  }
      	else
      	  {
      	    Entity::commandLine( cmdLine,cmdArgs,os );
      	  }
      }

      /* --------------------------------------------------------------------- */
      /* --- HELPER FUNCTIONS ------------------------------------------------ */
      /* --------------------------------------------------------------------- */


      /** Find the two minimum values according to some dimension, and return their indexes 
	  \param [in] points matrix containing the points (3x4)
	  \param [in] dimension can be 0,1,2 (for x,y or z)
	  \param [out] imin1 minimum index 1
	  \param [out] imin2 minimum index 2
       */
      void FootCollisionGrid::
      calculateTwoMins(const Eigen::MatrixXd & points, int dimension, 
			    unsigned int & imin1, unsigned int & imin2)
      {
	imin1=0, imin2=1;
	double min1=points(dimension, imin1), min2=points(dimension, imin2);
	
	for(int i=2; i<4; i++) {
	  if(points(dimension,i)<min1 && points(dimension,i)<min2){
	    if (min1>min2){
	      min1 = points(dimension,i);
	      imin1 = i;
	    }
	    else{
	      min2 = points(dimension,i);
	      imin2 = i;
	    }
	  }
	  else if(points(dimension,i) < min1){
	    min1 = points(dimension, i);
	    imin1 = i;
	  }
	  else if(points(dimension,i) < min2){
	    min2 = points(dimension, i);
	    imin2 = i;
	  }
	}

      }


      /** Initialize the grid finding the initial point for the grid: the backmost and 
	  rightmost point on the sole of the foot (very likely to be outside the foot 
	  to allow for a uniform grid starting from the middle of the foot)
       */
      void FootCollisionGrid::
      initializeGridWrtAnkle(void)
      {
	// Find the half-size of the grid (in x,y)
	int halfGridX = int(ceil((centerWrtAnkle(0)-gridStepSize/2-cornersWrtAnkle(0,0))/gridStepSize));
	int halfGridY = int(ceil((centerWrtAnkle(1)-gridStepSize/2-cornersWrtAnkle(1,0))/gridStepSize));

	// Find the foot origin (assumes z=constant) in homogeneous coordinates
	originWrtAnkle(0) = centerWrtAnkle(0)-gridStepSize/2 - double(halfGridX)*gridStepSize;
	originWrtAnkle(1) = centerWrtAnkle(1)-gridStepSize/2 - double(halfGridY)*gridStepSize; 
	originWrtAnkle(2) = centerWrtAnkle(2);
	originWrtAnkle(3) = 1.0;
	DEBUG_MSG("originWrtAnkle = [" << originWrtAnkle.transpose() << "];" << std::endl);

	// Size of the grid;
	int sizeX = 2*halfGridX + 1;
	int sizeY = 2*halfGridY + 1;
	DEBUG_MSG("Size of grid in x = " << sizeX << ", in y = " << sizeY << std::endl);
	
	std::pair<int,int> indexGrid;
	for(int x=0; x<sizeX; x++){
	  for(int y=0; y<sizeY; y++){
	    indexGrid.first  = x;
	    indexGrid.second = y;
	    grid[indexGrid] = 0;
	  }
	}

      }

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph

