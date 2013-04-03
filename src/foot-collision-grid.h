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

/*! \file foot-collision-grid.h
  \brief It is an specialization for the robot foot that simulates a grid on the 
  sole to discretize the collision points detected. Discretization is an alternative
  to eliminate numerical noise and instabilities, and it is the way a real array of
  sensors would be used on the sole.
*/


#ifndef __sot_oscar_FootCollisionGrid_H__
#define __sot_oscar_FootCollisionGrid_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dynamic_interpretor_EXPORTS)
#    define SOTFOOTCOLLISIONGRID_EXPORT __declspec(dllexport)
#  else
#    define SOTFOOTCOLLISIONGRID_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFOOTCOLLISIONGRID_EXPORT
#endif

/* ------------------------------------------------------------------
--- INCLUDE ---------------------------------------------------------
--------------------------------------------------------------------- */

// SOT
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <Eigen/Dense>

/* For GCAL: Convex hull */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>


namespace dynamicgraph {
  namespace sot {
    namespace oscar {

      /* ---------------------------------------------------------------------
         --- CLASS -----------------------------------------------------------
         --------------------------------------------------------------------- */

      class SOTFOOTCOLLISIONGRID_EXPORT FootCollisionGrid
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<FootCollisionGrid>
	{
	  
	public:
	  /* --- CONSTRUCTOR ---- */
	  FootCollisionGrid( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const {return CLASS_NAME;}
	  virtual void commandLine( const std::string& cmdLine,
				    std::istringstream& cmdArgs,
				    std::ostream& os );
	  void initCommands( void );

	  /* --- SIGNALS --- */
	  /*! \brief Signal [input]: Input contact points (to be discretized) */
	  DECLARE_SIGNAL_IN(contactPointsIN, ml::Matrix);

	  /*! \brief Signal [input]: Homogeneous transformation of the foot */
	  DECLARE_SIGNAL_IN(footTransf, ml::Matrix);

	  /*! \brief Signal [output]: Outputs the size of each step of the grid */
	  DECLARE_SIGNAL_OUT(gridStepSize, double);

	  /*! \brief Signal [output]: Coordinates of the foot corner, with respect to its ankle */
	  DECLARE_SIGNAL_OUT(footCornersWrtAnkle, ml::Matrix);

	  /*! \brief Signal [input]: Coordinales of the foot corner, with respect to the world frame */
	  DECLARE_SIGNAL_OUT(footCornersWrtWorld, ml::Matrix);

	  /*! \brief Signal [output]: discretized contact points according to the grid (all the points) */
	  DECLARE_SIGNAL_OUT(discContactPointsFull, ml::Matrix); 

	  /** Signal [output]: discretized contact points according to the grid (reduced set containing
	      only the convex hull */
	  DECLARE_SIGNAL_OUT(discContactPoints, ml::Matrix); 


	  /* --- COMMANDS --- */
	  void setGridStepSize( const double & stepSize );
	  void setFootCorners( const ml::Matrix & footCorners );

	  void cmd_printInformation( void );

	private:

	  /* Typedefs for cgal */
	  typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
	  typedef K::Point_2  Point_2;
	  typedef std::vector<Point_2> Points;

	  /* --- Variables --- */
	  double gridStepSize;
	  Eigen::MatrixXd cornersWrtAnkle;        // Corners of the foot with respect to the ankle
	  Eigen::MatrixXd cornersWrtWorld;        // Corners of the foot with respect to the world
	  Eigen::Vector4d originWrtAnkle;         // Virtual back right point of the foot (wrt ankle)
	  Eigen::Vector4d centerWrtAnkle;         // Center of the foot with respect to ankle 

	  std::map<std::pair<int,int>, int> grid; // Grid int pairs that indicate grid parts in collision
	  std::vector<Eigen::Vector4d> gridContactPoints; // Contact Points in the grid

	  Points cgalPoints, cgalResult; 

	  /* --- Helper Functions --- */
	  void calculateTwoMins(const Eigen::MatrixXd & points, int dimension, 
				unsigned int & imin1, unsigned int & imin2);
	  void initializeGridWrtAnkle( void );

	}; // class FootCollisionGrid

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_oscar_FootCollisionGrid_H__
