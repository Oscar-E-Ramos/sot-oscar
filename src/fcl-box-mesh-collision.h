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

/*! \file fcl-box-mesh-collision.h
  \brief Wraps the collision detection between a box (defined by its dimensions)
  and a mesh (in format .obj) using fcl. For obtaining the convex hull of the 
  points, it uses CGAL.
*/


#ifndef __sot_oscar_FclBoxMeshCollision_H__
#define __sot_oscar_FclBoxMeshCollision_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dynamic_interpretor_EXPORTS)
#    define SOTFCLBOXMESHCOLLISION_EXPORT __declspec(dllexport)
#  else
#    define SOTFCLBOXMESHCOLLISION_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFCLBOXMESHCOLLISION_EXPORT
#endif

/* ------------------------------------------------------------------
--- INCLUDE ---------------------------------------------------------
--------------------------------------------------------------------- */

// SOT
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <Eigen/Dense>

/* For FCL collision detection */
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"

/* For GCAL: Convex hull */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>



namespace dynamicgraph {
  namespace sot {
    namespace oscar {

      /* ---------------------------------------------------------------------
         --- CLASS -----------------------------------------------------------
         --------------------------------------------------------------------- */

      class SOTFCLBOXMESHCOLLISION_EXPORT FclBoxMeshCollision
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<FclBoxMeshCollision>
	{
	  
	public:
	  /* --- CONSTRUCTOR ---- */
	  FclBoxMeshCollision( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const {return CLASS_NAME;}
	  virtual void commandLine( const std::string& cmdLine,
				    std::istringstream& cmdArgs,
				    std::ostream& os );
	  void initCommands( void );

	  /* --- SIGNALS --- */

	  /*! \brief Signal [input]: Homogeneous Transformation for the box */
	  DECLARE_SIGNAL_IN(transfBoxIN, ml::Matrix);

	  /*! \brief Signal that returns the tolerance for the minimum distance between close points */
	  DECLARE_SIGNAL_OUT(minDistance, double);

	  /*! \brief Signal that returns the tolerance for the minimum distance between close points */
	  DECLARE_SIGNAL_OUT(maxNumContacts, int);

	  /** Signal: current transformation of the box (3x4 matrix: [R|T]) */
	  DECLARE_SIGNAL_OUT(transfBox, ml::Matrix); 

	  /** Signal: current transformation of the mesh (3x4 matrix: [R|T]) */
	  DECLARE_SIGNAL_OUT(transfMesh, ml::Matrix); 

	  /** Signal: points in contact (nx3 matrix) */
	  DECLARE_SIGNAL_OUT(contactPoints, ml::Matrix);


	  /* --- COMMANDS --- */
	  void setMinDistance( const double & minimumDistance );
	  void setMaxNumContacts( const int & maxNumContacts );
	  void setBoxDimensions( const ml::Vector & dim ); 
	  void setBoxPose( const ml::Vector & pose );
	  void setBoxTransformation( const ml::Matrix & M );
	  void setMeshFile( const std::string & meshFile ); 
	  void setMeshPose( const ml::Vector & pose );
	  void setMeshTransformation( const ml::Matrix & M );

	private:

	  /* Typedefs for cgal */
	  typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
	  typedef CGAL::Polyhedron_3<K>  Polyhedron_3;
	  typedef K::Segment_3           Segment_3;
	  typedef K::Triangle_3          Triangle_3;
	  typedef K::Point_3             Point_3;

	  /* --- Variables --- */
	  double distTolerance;
	  int num_max_contacts;

	  // For the box
	  fcl::Box box;
	  fcl::Transform3f transformBox;
	  // For the mesh
	  fcl::BVHModel<fcl::OBB>    mesh;
	  std::vector<fcl::Vec3f>    meshPoints;
	  std::vector<fcl::Triangle> meshTriang;
	  fcl::Transform3f transformMesh;

	  // For the convex hull (cgal)
	  CGAL::Object ch_object;
	  Polyhedron_3 poly; 
	  Point_3      point;
	  Segment_3    segment;
	  Triangle_3   triangle;
	  std::vector<Point_3> cgalPoints;

	  /* --- Structure for the points --- */
	  typedef struct { 
	    double x; double y; double z;
	  } Points; 
	  

	  /* --- Helper Functions --- */
	  void rpyToMatrix(double r, double p, double y, fcl::Matrix3f& R);
	  void loadOBJFile(const char* filename,
			   std::vector<fcl::Vec3f>& points,
			   std::vector<fcl::Triangle>& triangles);
	  void setMfromRT(ml::Matrix & mlM, const fcl::Matrix3f &R, const fcl::Vec3f &T);
	  void eliminateDuplicates(const std::vector<fcl::Contact> &contacts, 
				   std::vector<Points> &contactPoints, 
				   double dist_tolerance); 
	  int myConvexHull( std::vector<Eigen::Vector3d>& points, 
			    double minPointsDistance );
	  void eliminateClosePoints(std::vector<Eigen::Vector3d> &points, 
				    double dist_tolerance) ;


	}; // class FclBoxMeshCollision

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_oscar_FclBoxMeshCollision_H__
