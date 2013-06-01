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


#include <sot-oscar/fcl-box-mesh-collision.h>
#include <sot-dyninv/commands-helper.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FclBoxMeshCollision,"FclBoxMeshCollision");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      FclBoxMeshCollision::
      FclBoxMeshCollision( const std::string & name )
	: Entity(name)
	,distTolerance(0.002)
	,num_max_contacts(std::numeric_limits<int>::max())
	,CONSTRUCT_SIGNAL_IN(transfBoxIN, ml::Matrix)
	,CONSTRUCT_SIGNAL_OUT(minDistance, double, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(transfBox, ml::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(transfMesh, ml::Matrix, sotNOSIGNAL)	  
	,CONSTRUCT_SIGNAL_OUT(contactPoints, ml::Matrix, 
			      transfBoxINSIN )
	,CONSTRUCT_SIGNAL_OUT(maxNumContacts, int, sotNOSIGNAL)
      {
	signalRegistration(minDistanceSOUT << transfBoxSOUT << transfMeshSOUT
			   << contactPointsSOUT << maxNumContactsSOUT
			   << transfBoxINSIN);
	minDistanceSOUT.setNeedUpdateFromAllChildren( true );
	transfBoxSOUT.setNeedUpdateFromAllChildren( true );
	transfMeshSOUT.setNeedUpdateFromAllChildren( true );
	//contactPointsSOUT.setNeedUpdateFromAllChildren( true );
	maxNumContactsSOUT.setNeedUpdateFromAllChildren( true );
	initCommands();
      }

      void FclBoxMeshCollision::
      initCommands( void )
      {
	using namespace dynamicgraph::command;

	addCommand("setMinDistance",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setMinDistance,
				    dyninv::docCommandVoid1("Set the min distance between close points.",
							    "minDistance (double)")));

	addCommand("setMaxNumContacts",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setMaxNumContacts,
				    dyninv::docCommandVoid1("Set the max number of possible contacts.",
							    "numMaxContacts (int)")));
	
	addCommand("setBoxDimensions",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setBoxDimensions,
				    dyninv::docCommandVoid1("Set the dimensions of the box",
							    "Dimensions in x, y, z (vector)")));

	addCommand("setBoxPose",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setBoxPose,
				    dyninv::docCommandVoid1("Set the position and orientation (roll, pitch, yaw) of the box",
							    "Vector of pose (x,y,z,roll,pitch,yaw)")));

	addCommand("setBoxTransf",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setBoxTransformation,
				    dyninv::docCommandVoid1("Set the position and orientation of the box",
							    "Homogeneous transformation matrix (4x4)")));

	addCommand("setMeshFile",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setMeshFile,
				    dyninv::docCommandVoid1("Set the file to be used as mesh",
							    "Path to file in .obj format (string)")));

	addCommand("setMeshPose",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setMeshPose,
				    dyninv::docCommandVoid1("Set the position and orientation (roll, pitch, yaw) of the mesh",
							    "Vector of pose (x,y,z,roll,pitch,yaw)")));

	addCommand("setMeshTransf",
		   makeCommandVoid1(*this, &FclBoxMeshCollision::setMeshTransformation,
				    dyninv::docCommandVoid1("Set the position and orientation of the mesh",
							    "Homogeneous transformation matrix (4x4)")));


      }
      
      /* ---------------------------------------------------------------------- */
      /* --- COMMANDS --------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /** Sets the minimum distance between close points (to avoid duplicate points)
	  \param [in] minimumDistance minimum distance
      */
      void FclBoxMeshCollision::
      setMinDistance( const double & minimumDistance )
      {
	distTolerance = minimumDistance;
      }

      /** Sets the maximum number of points that can be detected in collision
	  \param [in] numMaxContacts maximum number of possible contacts
      */
      void FclBoxMeshCollision::
      setMaxNumContacts( const int & maxNumContacts )
      {
	num_max_contacts = maxNumContacts;
      }

      /*! \brief Sets the box dimensions
	\param [in] dim Dimensions of the box \f$(L_x, L_y, L_z)\f$ assuming that it has its
        center at \f$(0, 0, 0)\f$.
      */
      void FclBoxMeshCollision::
      setBoxDimensions( const ml::Vector & dim )
      {
      	box.side.setValue(dim(0), dim(1), dim(2));
      }

      /*! \brief Sets the transformation of the box using a homogeneous transformation matrix
        \param [in] M Transformation Matrix (4x4) specifying the translation and rotation of the box 
      */
      void FclBoxMeshCollision::
      setBoxTransformation( const ml::Matrix & M )
      {
      	fcl::Matrix3f R(M(0,0), M(0,1), M(0,2),
      			M(1,0), M(1,1), M(1,2),
      			M(2,0), M(2,1), M(2,2) );
      	fcl::Vec3f T(M(0,3), M(1,3), M(2,3));

      	transformBox.setTransform(R, T);
      }

      /*! \brief Sets the transformation of the box using the position and orientation (in roll, pitch, 
	yaw angles)
	\param [in] pose Vector containing the initial position and orientation of the box. The position
	is in cartesian coordinates and the orientation in roll, pitch, yaw angles:
	\f$(p_x,p_y,p_z,\theta_r,\theta_p,\theta_y)\f$, such that
	\f$R=R_z(\theta_r)R_y(\theta_p)R_x(\theta_y)\f$ 
       */
      void FclBoxMeshCollision::
      setBoxPose( const ml::Vector & pose )
      {
	fcl::Vec3f T(pose(0), pose(1), pose(2));
	fcl::Matrix3f R;
	rpyToMatrix(pose(3), pose(4), pose(5), R);
	transformBox.setTransform(R, T);
      }

      /*! \brief Sets the path to the file containing the mesh (in .obj format)
	\param [in] meshFile Path to the file (string)
      */
      void FclBoxMeshCollision::
      setMeshFile( const std::string & meshFile )
      {
	meshPoints.clear();
	meshTriang.clear();

	loadOBJFile(meshFile.c_str(), meshPoints, meshTriang);
	fcl::SplitMethodType split_method = fcl::SPLIT_METHOD_BV_CENTER;
	mesh.bv_splitter.reset(new fcl::BVSplitter<fcl::OBB>(split_method));
	
	mesh.beginModel();
	mesh.addSubModel(meshPoints, meshTriang);
	mesh.endModel();
      }

      /*! \brief Sets the transformation of the mesh using a homogeneous transformation matrix
        \param [in] M Transformation Matrix (4x4) specifying the translation and rotation of the mesh 
      */
      void FclBoxMeshCollision::
      setMeshTransformation( const ml::Matrix & M )
      {
      	fcl::Matrix3f R(M(0,0), M(0,1), M(0,2),
      			M(1,0), M(1,1), M(1,2),
      			M(2,0), M(2,1), M(2,2) );
      	fcl::Vec3f T(M(0,3), M(1,3), M(2,3));
	
      	transformMesh.setTransform(R, T);
      }

      /*! \brief Sets the transformation of the mesh using the position and orientation (in roll, pitch, 
	yaw angles)
	\param [in] pose Vector containing the initial position and orientation of the mesh. The position
	is in cartesian coordinates and the orientation in roll, pitch, yaw angles:
	\f$(p_x,p_y,p_z,\theta_r,\theta_p,\theta_y)\f$, such that
	\f$R=R_z(\theta_r)R_y(\theta_p)R_x(\theta_y)\f$ 
       */
      void FclBoxMeshCollision::
      setMeshPose( const ml::Vector & pose )
      {
	fcl::Vec3f T(pose(0), pose(1), pose(2));
	fcl::Matrix3f R;
	rpyToMatrix(pose(3), pose(4), pose(5), R);
	transformMesh.setTransform(R, T);
      }


      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      double& FclBoxMeshCollision::
      minDistanceSOUT_function( double& minDistance_, int t)
      {
      	minDistance_ = distTolerance;
      	return minDistance_;
      }

      int& FclBoxMeshCollision::
      maxNumContactsSOUT_function( int& maxNumContacts_, int t)
      {
      	maxNumContacts_ = num_max_contacts;
      	return maxNumContacts_;
      }

      ml::Matrix& FclBoxMeshCollision::
      contactPointsSOUT_function( ml::Matrix& mlpoints, int t )
      {
	static const bool enable_contact = true;
	fcl::CollisionRequest request(num_max_contacts, enable_contact);
	fcl::CollisionResult  result;
	result.clear();
	cgalPoints.clear();

	/* Get the transformation for the box from the signal */
	const ml::Matrix & M = transfBoxINSIN(t);
      	fcl::Matrix3f R(M(0,0), M(0,1), M(0,2),
      			M(1,0), M(1,1), M(1,2),
      			M(2,0), M(2,1), M(2,2) );
      	fcl::Vec3f T(M(0,3), M(1,3), M(2,3));
      	transformBox.setTransform(R, T);

	/* Detect the collision with fcl and get the information */
	collide(&box, transformBox, &mesh, transformMesh, request, result);
	
	/* If there were points in contact */
	if(result.numContacts() > 0) {
	  
	  std::vector<fcl::Contact> contacts;
	  result.getContacts(contacts);
	  
	  /* Create a vector containing a structure of Points to sort them and eliminate duplicates */
	  std::vector<Points> contactPoints;
	  double duplicateTolerance = 1e-6;
	  eliminateDuplicates(contacts, contactPoints, duplicateTolerance); 

	  /* Store results and save the points in a format suitable for cgal*/
	  for (unsigned int i=0; i<contactPoints.size(); ++i){
	    Point_3 p(contactPoints[i].x, contactPoints[i].y, contactPoints[i].z);
	    cgalPoints.push_back(p);
	  }    
	  
	  /* Compute the 3D convex hull of the points using CGAL*/
	  CGAL::Convex_hull_traits_3<K> traits;
	  CGAL::convex_hull_3(cgalPoints.begin(), cgalPoints.end(), ch_object, traits);

	  /* To store the contact points, if necessary */
	  std::vector<Eigen::Vector3d> pointsCh;

	  /* If the object is a point: copy it to the mal directly and end the program */
	  if (assign(point, ch_object)) {
	    DEBUG_MSG("It is a point" << std::endl);
	    mlpoints.resize(1, 3);
	    mlpoints(0,0) = point[0];
	    mlpoints(0,1) = point[1];
	    mlpoints(0,2) = point[2];
	    return mlpoints;
	  }

	  /* If the object is a segment: copy it to the mal and end the program */
	  else if (assign(segment, ch_object)) {
	    DEBUG_MSG("It is a segment" << std::endl);
	    mlpoints.resize(2, 3);
	    for(unsigned int ii=0; ii<2; ii++) {
	      mlpoints(ii,0) = segment.point(ii)[0];
	      mlpoints(ii,1) = segment.point(ii)[1];
	      mlpoints(ii,2) = segment.point(ii)[2];
	    }
	    return mlpoints;
	  }

	  /* If the object is a triangle: copy it to the mal and end the program */
	  else if (assign(triangle, ch_object)) {
	    DEBUG_MSG("It is a triangle" << std::endl);
	    mlpoints.resize(3, 3);
	    for(unsigned int ii=0; ii<3; ii++) {
	      mlpoints(ii,0) = triangle[ii][0];
	      mlpoints(ii,1) = triangle[ii][1];
	      mlpoints(ii,2) = triangle[ii][2];
	    }
	    return mlpoints;
	  }

	  /* If the object is a polyhedron: copy it to the mal and end the program */
	  else if (assign(poly, ch_object)) {
	    DEBUG_MSG("It is a polyhedron" << std::endl);
	    mlpoints.resize(poly.size_of_vertices(), 3);
	    unsigned int ii=0;
	    for(Polyhedron_3::Vertex_iterator viter = poly.vertices_begin();
		viter!=poly.vertices_end(); viter++) {
	      mlpoints(ii,0) = viter->point()[0];
	      mlpoints(ii,1) = viter->point()[1];
	      mlpoints(ii,2) = viter->point()[2];
	      ii++;
	    }
	    return mlpoints;
	  }

	  /* No convex hull found with cgal: this is an error */ 
	  else{
	    std::cerr << "No convex hull detected by cgal" << std::endl;
	  }

	}
	
	/* If there were no points in contact */
	else {
	  mlpoints.resize(0,0);
	}

      	return mlpoints;
      }


      ml::Matrix& FclBoxMeshCollision::
      transfBoxSOUT_function( ml::Matrix& mlM1out, int t )
      {
	fcl::Matrix3f R_ = transformBox.getRotation();
	fcl::Vec3f    T_ = transformBox.getTranslation();
	
      	mlM1out.resize(3,4);
      	setMfromRT(mlM1out, R_, T_);
      	return mlM1out;
      }


      ml::Matrix& FclBoxMeshCollision::
      transfMeshSOUT_function( ml::Matrix& mlM2out, int t )
      {
	fcl::Matrix3f R_ = transformMesh.getRotation();
	fcl::Vec3f    T_ = transformMesh.getTranslation();
	
      	mlM2out.resize(3,4);
      	setMfromRT(mlM2out, R_, T_);
      	return mlM2out;
      }


      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      void FclBoxMeshCollision::
      display( std::ostream& os ) const
      {
      	os << "FclBoxMeshCollision "<< getName() << "." << std::endl;
      }

      void FclBoxMeshCollision::
      commandLine( const std::string& cmdLine,
      		   std::istringstream& cmdArgs,
      		   std::ostream& os )
      {
      	if( cmdLine == "help" )
      	  {
      	    os << "FclBoxMeshCollision:\n"
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

      /** Convert roll, pitch, yaw angles to a rotation matrix. 
	  \param [in] r,p,y The roll, pitch and yaw angles 
	  \param [out] R The rotation matrix such that \f$R=R_z(r)R_y(p)R_x(y)\f$ 
      */
      void FclBoxMeshCollision::
      rpyToMatrix(double r, double p, double y, fcl::Matrix3f& R)
      {
	double c1 = cos(r), c2 = cos(p), c3 = cos(y);
	double s1 = sin(r), s2 = sin(p), s3 = sin(y);
	
	R.setValue( c1*c2, -s1*c3+c1*s2*s3,  s1*s3+c1*s2*c3,   
		    s1*c2,  c1*c3+s1*s2*s3, -c1*s3+s1*s2*c3,
		    -s2,        c2*s3,          c2*c3     );
      }
      

      void FclBoxMeshCollision::
      loadOBJFile(const char* filename, 
		  std::vector<fcl::Vec3f>& points, 
		  std::vector<fcl::Triangle>& triangles)
      {
	FILE* file = fopen(filename, "rb");
	if(!file) {
	  std::cerr << "Error: cannot find the file " << filename << "!" << std::endl;
	  exit(EXIT_FAILURE);
	}
	
	bool has_normal = false;
	bool has_texture = false;
	char line_buffer[2000];

	while(fgets(line_buffer, 2000, file)) {
	  
	  char* first_token = strtok(line_buffer, "\r\n\t ");
	  if(!first_token || first_token[0] == '#' || first_token[0] == 0)
	    continue;
	  
	  switch(first_token[0])
	    {
	    case 'v':
	      {
		if(first_token[1] == 'n') {
		  strtok(NULL, "\t ");
		  strtok(NULL, "\t ");
		  strtok(NULL, "\t ");
		  has_normal = true;
		}
		else if(first_token[1] == 't') {
		  strtok(NULL, "\t ");
		  strtok(NULL, "\t ");
		  has_texture = true;
		}
		else {
		  fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
		  fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
		  fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
		  fcl::Vec3f p(x, y, z);
		  points.push_back(p);
		}
	      }
	      break;
	    case 'f':
	      {
		fcl::Triangle tri;
		char* data[30];
		int n = 0;
		while((data[n] = strtok(NULL, "\t \r\n")) != NULL) {
		  if(strlen(data[n]))
		    n++;
		}

		for(int t = 0; t < (n - 2); ++t) {
		  if((!has_texture) && (!has_normal)) {
		    tri[0] = atoi(data[0]) - 1;
		    tri[1] = atoi(data[1]) - 1;
		    tri[2] = atoi(data[2]) - 1;
		  }
		  else {
		    const char *v1;
		    for(int i = 0; i < 3; i++) {
		      // vertex ID
		      if(i == 0)
			v1 = data[0];
		      else
			v1 = data[t + i];
		      
		      tri[i] = atoi(v1) - 1;
		    }
		  }
		  triangles.push_back(tri);
		}
	      }
	    }
	}
      }
      
      
      /** Set the transformation matrix M=[R|T] of size 3x4 from R and T */
      void FclBoxMeshCollision::
      setMfromRT(ml::Matrix & mlM, const fcl::Matrix3f &R, const fcl::Vec3f &T)
      {
      	mlM(0,0) = R(0,0); mlM(0,1) = R(0,1); mlM(0,2) = R(0,2); mlM(0,3) = T[0];
      	mlM(1,0) = R(1,0); mlM(1,1) = R(1,1); mlM(1,2) = R(1,2); mlM(1,3) = T[1];
      	mlM(2,0) = R(2,0); mlM(2,1) = R(2,1); mlM(2,2) = R(2,2); mlM(2,3) = T[2];
      }

      /*
	Eliminates the points that are closer than dist_tolerance using the manhattan distance
        if: |x1-x2|<d & |y1-y2|<d & |z1-z2|<d
      */
      void FclBoxMeshCollision::
      eliminateDuplicates(const std::vector<fcl::Contact> &contacts, 
			  std::vector<Points> &contactPoints, 
			  double dist_tolerance) 
      {
	unsigned int j;
	bool tooClose;
	
	/* Copy the first element */
	Points point1;
	point1.x = contacts[0].pos[0];
	point1.y = contacts[0].pos[1];
	point1.z = contacts[0].pos[2];
	contactPoints.push_back(point1);

	/* For loop */
	for(unsigned int i=1; i< contacts.size(); i++){
	  for(j=0; j< contactPoints.size(); j++)
	    {
	      /* Comparison to see if the points are close enough
		 using the 'Manhattan' distance */
	      tooClose = ( (fabs(contacts[i].pos[0] - contactPoints[j].x) < dist_tolerance) && 
			   (fabs(contacts[i].pos[1] - contactPoints[j].y) < dist_tolerance) && 
			   (fabs(contacts[i].pos[2] - contactPoints[j].z) < dist_tolerance) );
	      if (tooClose)
		break;
	    }

	  /* if none of the values in index[0..j] of array is not same as array[i],
	     then copy the current value to corresponding new position in array */
	  if (j==contactPoints.size() ){
	    point1.x = contacts[i].pos[0];
	    point1.y = contacts[i].pos[1];
	    point1.z = contacts[i].pos[2];
	    contactPoints.push_back(point1);
	  }
    
	}
      }  


      /** Find the convex hull of a set of points (on the plane that maximizes the area of the points).
	  The basic assumption is that all the points are coplanar (or close to coplanar within some 
	  tolerance, which is logic for HRP-2 foot)
	  \param [in/out] points The points before and after the convex hull processing
	  \param [in] minPointsDistance The minimum distance to assume that 2 points are 'similar' 
	              (it is a tolerance)
      */
      int FclBoxMeshCollision::
      myConvexHull( std::vector<Eigen::Vector3d>& points, 
		    double minPointsDistance )
      {
	/* If there is no (input) contact point, show an error and exit the function */
	if (points.size()<1) {
	  std::cerr << "There is no input point to process in myConvexHull" << std::endl;
	  return 0;
	}

	/* If there is only 1 contact point, discard the verification test and exit the function */
	if (points.size() < 2)
	  return 0;
  
	/* If there are two contact points check if they are close enough to be discarded. If so, 
	   return the mean of them. Otherwise, return both points                                */
	if (points.size() < 3)
	  {
	    if((fabs(points[0](0)-points[1](0)) < minPointsDistance) && 
	       (fabs(points[0](1)-points[1](1)) < minPointsDistance) &&
	       (fabs(points[0](2)-points[1](2)) < minPointsDistance) )
	      {
		Eigen::Vector3d Pmean = 0.5*(points[0]+points[1]);
		points.clear();
		points.push_back(Pmean);
		return 0;
	      }
	    return 0;
	  }

	/* Definition of tolerances */
	//tol_eliminateClosePoints = 1e-4;

	/* Check if the points are close enough. If so, pick the first point and discard the others that
	   are close to it     */
	// DEBUG_MSG("Before eliminating close points there were: "<<points.size()<<" points"<<std::endl);
	// eliminateClosePoints(points, minPointsDistance);
	// DEBUG_MSG("After eliminating close points there are: "<<points.size()<<" points"<<std::endl);
	// OUTPUT_PTS("pointsNotClose=[");
	// for(unsigned int i=0; i<points.size(); i++)
	//   OUTPUT_PTS(points[i].transpose() << ";");
	// OUTPUT_PTS("];" << std::endl);

	/* ---- Find the normal to the collision plane ----
	   Let the points be p1, p2, p3, p4, ...
	   - Use (p1, p2) to get the first vector 'v1'. 
	   - Use (p1, p3) to get the second vector 'v2'
	   - Find the 'normal' as the cross product of v1 x v2. If is different to zero (v1,v2 are not
	     collinear), then use the value as the normal. Otherwise, compute v2 using (p1,p4), then
	     (p1,p5), ... until the 'normal' is not zero (if possible). If the normal is always zero,
	     then, all the points lie on the same line and the extremes of the line have to be found
	*/

	double EPSILON = 1e-5; //minPointsDistance/10;    // 'zero': to verify if normal is equal to zero
	Eigen::Vector3d p3, v1, v2, normal;
	bool normalNotZero;
	v1 = points[0]-points[1];

	for(unsigned int i=2; i<points.size(); i++) {
	  v2 = points[0] - points[i];
	  normal = v1.cross(v2);
	  normalNotZero = fabs(normal(0)) > EPSILON || 
	    fabs(normal(1)) > EPSILON ||
	    fabs(normal(2)) > EPSILON;
	  if (normalNotZero)
	    break;
	}

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	                THE POINTS ARE COLLINEAR: FIND THE EXTREMES OF THE LINE
	   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	   The equation of the line passing through the points is P = p1 + t*v1, then, the extremes
	   wil be those points for which 't' is minimum and maximum. Find t as: t=(P-p1)/v1, where
	   only the first component (or second [or third], if the first [or second] is equal to zero) 
	   of the vectors is considered. We can just compare one component, since we know that the
	   points are collinear, and thus, t must be the same for any component different to zero      
	*/
	if (!normalNotZero)
	  {
	    int ind=0;                // Index for v1 (and the other vectors)
	    int imax=0, imin=0;       // Indexes corresponding to the max and min of 't'
	    double tmax=-1e10, tmin=1e10;    // Maximum and minimum values of 't' (initially 0 since P=p1)
	    double ttemp;             // Temporal value to store 't'

	    if (fabs(v1(ind))<minPointsDistance){
	      ind=1;
	      if (fabs(v1(ind))<minPointsDistance){
		ind=2;
	      }
	    }
 
	    /* Find the maximum and minimum t */
	    for (unsigned int j=0; j<points.size(); j++) {
	      ttemp = ( points[j](ind) - points[1](ind) )/v1(ind);
	      if (ttemp>tmax){
		tmax=ttemp; imax=j;
	      }
	      else if(ttemp<tmin){
		tmin=ttemp; imin=j;
	      }
	    }
      
	    /* Create a copy with the same elements as points (collision points) */
	    std::vector<Eigen::Vector3d> pointsTemp;
	    pointsTemp = points;
	    points.clear();
	    /* Keep only the points corresponding to the extremes of the line */
	    points.push_back(pointsTemp[imin]);
	    points.push_back(pointsTemp[imax]);

	    return 0;
	  }
  

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	             THE POINTS FORM A POLYGON (IN A PLANE)
	   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	*/

	/* Treatment of the Normals: 
	   ========================
	   Project onto an axis-aligned plane that maximizes the area of the polygon. That is, discard
	   the largest normal component
	*/
	Eigen::Vector3d absN, tempNormal;
	int x, y, inot;
	double maxNorm;

	/* Find the maximum normal (within consecutive elements: (v20xv10), (v31xv32), ...  This is done
	   to avoid 'false' normals between very close points 
	*/
	v1 = points[2] - points[0];
	v2 = points[1] - points[0];
	normal = v1.cross(v2);
	maxNorm = normal.norm();

	for (unsigned int i=1; i<points.size()-2; i++){
	  v1 = points[i+2] - points[i];
	  v2 = points[i+1] - points[i];
	  tempNormal = v1.cross(v2);
	  if (tempNormal.norm() > maxNorm){
	    maxNorm = tempNormal.norm();
	    normal = tempNormal;
	  }
	}

	/* Use the absolute value of the normal components to discard the largest normal component: find the
	   other components to use them as axis 'x,y' and suppose the points are in that plane */
	absN(0) = fabs(normal(0)); absN(1)=fabs(normal(1)); absN(2)=fabs(normal(2));
	if( absN(0)>absN(1) ) {
	  if( absN(0)>absN(2) ) {
	    x=1; y=2;  inot=0;     /* absN[0] is greatest */
	  }
	  else {
	    x=0; y=1;  inot=2;     /* absN[2] is greatest */
	  }
	}
	else {                       /* absN[0]<=absN[1] */
	  if( absN(2)>absN(1) ) {
	    x=0; y=1;  inot=2;     /* absN[2] is greatest */
	  }
	  else {
	    x=0; y=2;  inot=1;     /* absN[1] is greatest */
	  }
	}               
	DEBUG_MSG("The 2D axis for projection are: x=" << x << ", y=" << y << std::endl);


	/* Find the lowest point in y and x for the algorithm to start
	   ===========================================================
	   It will be an extreme of the convex hull (even though it will not be used until the
	   end of the convex hull deteection)
	*/

	/* Find the minimum point in y direction. Points within a certain tolerance to the minimum
	   are considered. If there are many mins in the y axis (there is a minimal line), then, 
	   find the min in x within those points.
	*/

	double       ymin, xmin;
	unsigned int imin;
	std::vector<unsigned int> iminY;

	/* Search the min in the y direction 
	   <TOLERANCE>: minPointsDistance         */	
	iminY.push_back(0);
	ymin = points[0](y);
	for (unsigned int i=1; i<points.size(); i++) {
	  DEBUG_MSG("~ Point" << i << " with y = " << points[i](y) << std::endl);
	  if ( (points[i](y) < ymin) && ((ymin-points[i](y))>minPointsDistance) ){
	    ymin = points[i](y);
	    iminY.clear();
	    iminY.push_back(i);
	    DEBUG_MSG("  -> New ymin: "<< ymin <<", Point"<< i <<" added for checking xmin"<< std::endl);
	  }
	  else if( fabs(points[i](y)-ymin) < minPointsDistance) {
	    iminY.push_back(i);
	    DEBUG_MSG("  -> Point" << i << " has y close to ymin ... adding it to check xmin" << std::endl);
	  }
	}

	DEBUG_MSG("Candidates with the lowest y (ymin) within a tolerance of " << minPointsDistance
		  << " are:" << std::endl);
	for (unsigned int i=0; i<iminY.size(); i++)
	  DEBUG_MSG("point" << iminY[i] << ", ");
	DEBUG_MSG(std::endl);

	/* Search in the x direction:
	   <TOLERANCE>: NONE. The exact minimum is found without tolerance */
	xmin = points[iminY[0]](x);
	imin = iminY[0];
	for(unsigned int i=1; i<iminY.size(); i++){
	  if (points[iminY[i]](x) < xmin ){
	    xmin = points[iminY[i]](x);
	    imin = iminY[i];
	  }
	}

	DEBUG_MSG("Initial min point: Point" << imin << " with (xmin=" << points[imin](x) <<  ", ymin=" 
		  << ymin << ", zmin=" << points[imin](2) << ")" << std::endl);


	/* Compute the angle as an estimator of the desired edge. 
	   =======================================================
	   Starting with the minimum point in the 'x' direction, and assuming that we want to go through 
	   the convex hull in counterclockwise sense, the initial 'vector' is horizontal (1,0). Then, 
	   find the angles. The minimum angles (within a small tolerance between them so that roundoff 
	   errors and a tolerance is considered) are the candidates for the next vertex of the hull (in 
	   fact they lie in a line: the edge). Among those points with min angle, the one that is further 
	   from the current point is chosen as the next point in the convex hull. 
	*/
	
	/* <TOLERANCE> thTolerance: for the angles. 
	   <TOLERANCE> minCloseDistAroundCHPoint: to discard points too close from the current point
	*/

	double thTolerance = 4.0/180.0*M_PI;         // 0.04 ~ 2.3 degrees
	double minCloseDistAroundCHPoint = 1e-2;     // 1 cm, experimentally works well with 4 deg

	//double e1[2] = {1.0, 0.0};    // First edge (horizontal, since xmin and ymin were found)
	double e1[2] = {cos(-0.3), sin(-0.3)}; // First edge (close to horizontal, to allow for tolerance
	double e2[2];                 // Second edge
	double p0[2] = {points[imin](x), points[imin](y)};   // First Point (with min x,y)
	double pi[2];                                        // Second Point
	double mod_e1=1, mod_e2, th, thmin;   // |e1|, |e2|

	std::vector<Eigen::Vector3d> pointsCH;
  	std::vector <int> indCollinear;

	unsigned int indexCH = imin;
	std::vector<unsigned int> indicesCH;
	std::vector<double> ths;

	/* Search for the points in the convex hull. The loop ends when a point already in the convex
	   hull is detected again, that is, when the polygon is closed
	*/ 
	while(true) 
	{
	  DEBUG_MSG("\npoint" << indexCH << " = [" << p0[0] << ", " << p0[1] << "], edge1=[" << e1[0]
		    << ", " << e1[1] << "];" << std::endl);
	  
	  thmin = M_PI;               // Maximum possible value (the angles vary from 0-180)
	  ths.clear();

	  /* Starting from the initial point p0, find the angle between the current edge (formed by p0pi
	     where pi is a point in the whole set) and the previous edge. Keep the points that have the
	     lowest values within the tolerance of thTolerance   	  */
	  for (unsigned int j=0; j<points.size(); j++)
	    {
	      if (j==indexCH)     // For each j!=i
		{
		  ths.push_back(0.0);     // Store the value of the angle (fictitious 0 angle)
		  continue;    
		}
	      pi[0]=points[j](x); pi[1]=points[j](y);    // Point to test
	      e2[0]=pi[0]-p0[0];  e2[1]=pi[1]-p0[1];     // edge from p0 to pi
	      mod_e2 = sqrt(e2[0]*e2[0]+e2[1]*e2[1]);    // |e2|
	      th = acos( (e2[0]*e1[0]+e2[1]*e1[1])/(mod_e2*mod_e1) );      // theta
	      ths.push_back(th);                         // Store the value of theta 

	      DEBUG_MSG("~ point"<<j<<"=["<<pi[0]<<", "<<pi[1]<<"]; e2=["<<e2[0]<<", "<<e2[1]<< "]; "
			<< "th=" << th*180/M_PI << std::endl);

	      /* Find the smallest angle which will give the direction for the next vertex of the polygon. 
		 An additional condition on the tolerance is imposed so that an angle smaller but within
		 the tolerance is not considered as smaller (but 'equal')         	      */
	      
	      if((th<thmin) && ((thmin-th)>thTolerance)){
		/* Discard the point if it is too close from the current point */
		if((fabs(pi[0]-p0[0]) > minCloseDistAroundCHPoint) ||
		   (fabs(pi[1]-p0[1]) > minCloseDistAroundCHPoint))
		  {
		    thmin = th;
		    indCollinear.clear();
		    indCollinear.push_back(j);
		    DEBUG_MSG("  -> *** New thmin ***: " << thmin*180/M_PI << ". Point" << j 
			      << " added for collinear test" << std::endl);	      
		  }
		else
		  DEBUG_MSG("  -> Point"<<j<<"is too close from current CH point. Not added for collinear"
			    << " test (tolerance=" << minCloseDistAroundCHPoint << ")" << std::endl);	      
	      }

	      /* Due to rounding errors and IMPOSED tolerance, angles with a small difference (given by
		 thTolerance) are considered as the same angle. This 'IF' finds angles that are close 
		 enough to the current minumum angle */
	      else if( fabs(th-thmin) < thTolerance ){
		indCollinear.push_back(j);
		DEBUG_MSG("  -> Point" << j << " is close to thmin" << std::endl);
		DEBUG_MSG("  -> Index: " << j << " added for collinear test" << std::endl);
	      }
	    }

	  DEBUG_MSG("Points with minimum angle:" << std::endl);
	  for (unsigned int i=0; i<indCollinear.size(); i++)
	    DEBUG_MSG(" point" << indCollinear[i] << ", ");
	  DEBUG_MSG(std::endl);

	  // /* The point with the maximum distance from the current point is searched among the points with
	  //    minimum angle. This will be added to the convex hull */
	  // if( !indCollinear.empty() ){
	  //   double maxDistance = 0.0, dist;
	  //   int ii;
	  //   for (unsigned int cnt=0; cnt<indCollinear.size(); cnt++){
	  //     ii = indCollinear[cnt];
	  //     dist = (p0[0]-points[ii](x))*(p0[0]-points[ii](x)) + 
	  // 	     (p0[1]-points[ii](y))*(p0[1]-points[ii](y));
	  //     if (dist > maxDistance){
	  // 	maxDistance = dist;
	  // 	maxDistIndex = ii;
	  //     }
	  //   }
	  // }


	  /* The point with the maximum distance within a tolerance from the current point is searched
	     among the points with minimum angle. A tolerance for the distance is considered. This will
	     be added to the convex hull
	  */

	  double minCloseDistLongestPoints = minPointsDistance/2;
	  if( !indCollinear.empty() ){
	    std::vector<unsigned int> indCollinearFar;
	    double maxDistance = 0.0, dist;
	    unsigned int ind;
	    for (unsigned int cnt=0; cnt<indCollinear.size(); cnt++){
	      ind = indCollinear[cnt];
	      dist = (p0[0]-points[ind](x))*(p0[0]-points[ind](x)) + 
	  	     (p0[1]-points[ind](y))*(p0[1]-points[ind](y));
	      DEBUG_MSG("Point" << ind << " has distance=" << dist << std::endl);
	      if ((dist > maxDistance) && ((dist-maxDistance) > minCloseDistLongestPoints) ){
	  	maxDistance = dist;
	  	indCollinearFar.clear();
		indCollinearFar.push_back(ind);
		DEBUG_MSG("  -> New maxDistance = " << maxDistance << ". Point" << ind 
			  << " added for proximity check, th="<<ths[ind]/M_PI*180<<std::endl);	      

	      }
	      else if( fabs(dist-maxDistance)<minCloseDistLongestPoints){
		indCollinearFar.push_back(ind);
		DEBUG_MSG("  -> Point" << ind << " has distance close enough to maxDistance within the "
			  "tolerance of: "<<minCloseDistLongestPoints<<", th="<<ths[ind]/M_PI*180
			  << std::endl);
	      }
	    }
	    thmin = M_PI;
	    //unsigned int indexCH;
	    for (unsigned int cnt=0; cnt<indCollinearFar.size(); cnt++){
	      if(ths[indCollinearFar[cnt]]<thmin){
		thmin = ths[indCollinearFar[cnt]];
		indexCH = indCollinearFar[cnt];
	      }
	    }
	  }

	  else
	    std::cerr << "ERROR: No following point for the convex hull was found (a.k.a. HELP!!!)" 
		      << std::endl;

	  /* Chech if the last point has already been added (close the polygon). If it has, 
	     end the function */
	  for(unsigned int i=0; i<indicesCH.size(); i++){
	    if (indexCH == indicesCH[i]){
	      points = pointsCH;
	      
	      OUTPUT_PTS("pointsCH=[");
	      for(unsigned int jj=0; jj<pointsCH.size(); jj++)
		OUTPUT_PTS(pointsCH[jj].transpose() << ";");
	      OUTPUT_PTS("];" << std::endl);

	      return 0;
	    }
	  }

	  pointsCH.push_back(points[indexCH]);
	  indicesCH.push_back(indexCH);
	  DEBUG_MSG("  point" << indexCH << " has been added to the convex hull" << std::endl);
	  DEBUG_MSG("  There are " << pointsCH.size() << " points in the convex hull." << std::endl);

	  //i = indexCH;
	  e1[0]=points[indexCH](x)-p0[0]; e1[1]=points[indexCH](y)-p0[1];
	  mod_e1 = sqrt(e1[0]*e1[0]+e1[1]*e1[1]);                      // |e2|
	  p0[0]=points[indexCH](x); p0[1]=points[indexCH](y);

	}
	  //} while(i!=imin);
	
	
	// points = pointsCH;
	return 0;

      }
      

      /** Eliminates the points that are closer than minDistTolerance using the manhattan distance,
	  that is, if \f$|x_1-x_2|<d \& |y_1-y_2|<d \& |z_1-z_2|<d\f$
	  \param [in/out] points Input and output points
	  \param dist_tolerance Maximum tolerance to consider that points are equal
      */
      void FclBoxMeshCollision::
      eliminateClosePoints(std::vector<Eigen::Vector3d> &points, 
			   double dist_tolerance) 
      {
	unsigned int j;
	bool tooClose;
  
	/* Copy the first element */
	Eigen::Vector3d point1;
	std::vector<Eigen::Vector3d> pointsOUT;
	point1 << points[0](0), points[0](1), points[0](2);
	pointsOUT.push_back(point1);

	/* For loop */
	for(unsigned int i=1; i< points.size(); i++){
	  for(j=0; j< pointsOUT.size(); j++)
	    {
	      /* Comparison to see if the points are close enough
		 using the 'Manhattan' distance */
	      tooClose = ( (fabs(points[i](0) - pointsOUT[j](0)) < dist_tolerance) && 
			   (fabs(points[i](1) - pointsOUT[j](1)) < dist_tolerance) && 
			   (fabs(points[i](2) - pointsOUT[j](2)) < dist_tolerance) );
	      if (tooClose)
		break;
	    }

	  /* if none of the values in index[0..j] of array is not same as array[i],
	     then copy the current value to corresponding new position in array */
	  if (j==pointsOUT.size() ){
	    point1 << points[i](0), points[i](1), points[i](2);
	    pointsOUT.push_back(point1);
	  }
	}
	points = pointsOUT;
      }  

    } // namespace oscar
  } // namespace sot
} // namespace dynamicgraph

