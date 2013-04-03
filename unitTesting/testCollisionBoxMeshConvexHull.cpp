/*
  2013 Oscar Efrain Ramos Ponce, LAAS-CNRS

  Test for the collision detection between one box and a mesh that represents
  a plane (in .ply format). Then, the convex hull is obtained using cgal.
*/


#include <iostream>
#include <stdio.h>

// For fcl collision detection
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"

// For using cgal
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Cartesian.h>
// #include <CGAL/Gmpq.h>

#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>


typedef struct { 
  double x; double y; double z;
} Points; 


// CGAL special types
typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
//typedef CGAL::Exact_predicates_exact_constructions_kernel  K;
// typedef CGAL::Gmpq                                      RT;
// typedef CGAL::Cartesian<RT>                             K;

typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Segment_3                              Segment_3;
typedef K::Triangle_3                             Triangle_3;
typedef K::Point_3                                Point_3;
typedef K::Vector_3                               Vector_3;
typedef CGAL::Creator_uniform_3<double, Point_3>  PointCreator;


void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles);
void generateTransform(fcl::FCL_REAL xyzrpy[6], fcl::Transform3f &transform);
void rpyToMatrix(fcl::FCL_REAL r, fcl::FCL_REAL p, fcl::FCL_REAL y, fcl::Matrix3f& R);
void eliminateDuplicates(const std::vector<fcl::Contact> &contacts, std::vector<Points> &contactPoints, double dist_tolerance); 


int main( void )
{
  /* Initialization of the files where the data will be stored */
  std::ofstream filePointsIn, filePointsOut;
  filePointsIn.open("/tmp/pointsIn");
  filePointsOut.open("/tmp/pointsOut");

  /* Vector containing the input data points */
  std::vector<Point_3> points;

  /* Define Object to hold the convex hull */
  CGAL::Object ch_object;
  Polyhedron_3 poly;
  Point_3 point;
  Segment_3 segment;

  /* Creation of the mesh */
  std::vector<fcl::Vec3f> p1;
  std::vector<fcl::Triangle> t1;
  loadOBJFile("plane2m.obj", p1, t1);

  fcl::BVHModel<fcl::OBB> m1;
  fcl::SplitMethodType split_method = fcl::SPLIT_METHOD_BV_CENTER;
  m1.bv_splitter.reset(new fcl::BVSplitter<fcl::OBB>(split_method));
  m1.beginModel(); m1.addSubModel(p1, t1); m1.endModel();

  fcl::Transform3f transformM1;
  fcl::FCL_REAL poseM1[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    /* x:0-2, y:0-2, z:0-0  */
  generateTransform(poseM1, transformM1);

  /* Creation and transformation of the cube */
  fcl::Box box1(2.0, 2.0, 2.0);
  fcl::FCL_REAL poseB1[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};  
  fcl::Transform3f transformB1;
  generateTransform(poseB1, transformB1);

  /* Collision parameters */
  static const int  num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact   = true;
  fcl::CollisionRequest request(num_max_contacts, enable_contact);    // In collision_data.h
  fcl::CollisionResult  result;    result.clear();

  /* Collision detection */
  int ncontacts = collide(&box1, transformB1, &m1, transformM1, request, result);

  if(result.numContacts() > 0) {

    std::vector<fcl::Contact> contacts;
    result.getContacts(contacts);

    std::vector<Points> contactPoints;
    eliminateDuplicates(contacts, contactPoints, 0.002); 

    /* Store results and save the points in a format suitable for cgal*/
    for (unsigned int i=0; i<contactPoints.size(); ++i){
      filePointsIn << contactPoints[i].x << " " << contactPoints[i].y << " " << contactPoints[i].z << "\n";
      Point_3 ptemp(contactPoints[i].x, contactPoints[i].y, contactPoints[i].z);
      points.push_back(ptemp);
    }    

    /* Compute the convex hull of the points */
    //CGAL::convex_hull_3(points.begin(), points.end(), ch_object);

    CGAL::Convex_hull_traits_3<K> traits;
    CGAL::convex_hull_3(points.begin(), points.end(), ch_object, traits);

    if (assign(point, ch_object)) {
      /* The object is a point */
      //filePointsOut << point << std::endl;
      std::cout << "The convex hull is a point" << std::endl;
    }

    else if (assign(segment, ch_object)) {
      /* The object is a segment*/
      filePointsOut << segment.point(0) << std::endl;
      filePointsOut << segment.point(1) << std::endl;
    }
    
    else if (assign(poly, ch_object)) {
      /* The object is a polyhedron*/
      for(Polyhedron_3::Vertex_iterator viter = poly.vertices_begin(); viter!=poly.vertices_end(); viter++) 
      	filePointsOut << viter->point() << std::endl;
      // for(Polyhedron_3::Point_iterator viter = poly.points_begin(); viter!=poly.points_end(); viter++) 
      // 	filePointsOut << viter[0] << std::endl;


    }

    

  }

  else {
    std::cout << "collision free " << std::endl;
  }

  filePointsIn.close();
  filePointsOut.close();

  return 0;
}


/* Generate the Transformation using the x,y,z and the roll, pitch, yaw
   Euler angles.
*/
void generateTransform(fcl::FCL_REAL xyzrpy[6], fcl::Transform3f &transform)
{
  fcl::Matrix3f R;
  rpyToMatrix(xyzrpy[3], xyzrpy[4], xyzrpy[5], R);
  fcl::Vec3f T(xyzrpy[0], xyzrpy[1], xyzrpy[2]);
  transform.setTransform(R, T);
}


/* Convert roll, pitch, yaw angles to a rotation matrix. The inputs are [r,p,y] and 
   the rotation is R = Rz(r)*Ry(p)*Rx(y)
 */
void rpyToMatrix(fcl::FCL_REAL r, fcl::FCL_REAL p, fcl::FCL_REAL y, fcl::Matrix3f& R)
{
  fcl::FCL_REAL c1 = cos(r), c2 = cos(p), c3 = cos(y),
                s1 = sin(r), s2 = sin(p), s3 = sin(y);

  R.setValue( c1*c2, -s1*c3+c1*s2*s3,  s1*s3+c1*s2*c3,   
	      s1*c2,  c1*c3+s1*s2*s3, -c1*s3+s1*s2*c3,
	        -s2,        c2*s3,          c2*c3     );
}


/* Load .obj files
 */
void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file) {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
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
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
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


/*
  Eliminates the points that are closer than dist_tolerance using the manhattan distance
        if: |x1-x2|<d & |y1-y2|<d & |z1-z2|<d
*/
void eliminateDuplicates(const std::vector<fcl::Contact> &contacts, std::vector<Points> &contactPoints, double dist_tolerance) 
{
  int j;
  bool tooClose;
  
  /* Copy the first element */
  Points point1;
  point1.x = contacts[0].pos[0];
  point1.y = contacts[0].pos[1];
  point1.z = contacts[0].pos[2];
  contactPoints.push_back(point1);

  /* For loop */
  for(int i=1; i< contacts.size(); i++){
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
