/*
  2013 Oscar Efrain Ramos Ponce, LAAS-CNRS

  Test for the collision detection between one box and a mesh that represents
  a plane (in .ply format)
*/

#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include <iostream>
#include <stdio.h>


typedef struct { 
  double x; double y; double z;
} Points; 


void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles);
void generateTransform(fcl::FCL_REAL xyzrpy[6], fcl::Transform3f &transform);
void rpyToMatrix(fcl::FCL_REAL r, fcl::FCL_REAL p, fcl::FCL_REAL y, fcl::Matrix3f& R);
void eliminateDuplicates(const std::vector<fcl::Contact> &contacts, std::vector<Points> &contactPoints, double dist_tolerance); 


int main( void )
{
  //fcl::GJKSolver_libccd solver1;
  //fcl::GJKSolver_indep solver2;

  /* For the mesh */
  std::vector<fcl::Vec3f> p1;
  std::vector<fcl::Triangle> t1;
  loadOBJFile("plane2m.obj", p1, t1);

  fcl::BVHModel<fcl::OBB> m1;
  //splitmethod: SPLIT_METHOD_MEAN, SPLIT_METHOD_BV_CENTER, SPLIT_METHOD_MEDIAN, 
  fcl::SplitMethodType split_method = fcl::SPLIT_METHOD_BV_CENTER;
  m1.bv_splitter.reset(new fcl::BVSplitter<fcl::OBB>(split_method));

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();

  fcl::Transform3f transformM1;
  fcl::FCL_REAL poseM1[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    /* x:0-2, y:0-2, z:0-0  */
  generateTransform(poseM1, transformM1);

  /* For the cube */
  fcl::Box box1(2.0, 2.0, 2.0);
  fcl::FCL_REAL poseB1[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};  
  fcl::Transform3f transformB1;
  generateTransform(poseB1, transformB1);

  /* Collision parameters */
  static const int  num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact   = true;
  fcl::CollisionRequest request(num_max_contacts, enable_contact);    // In collision_data.h
  fcl::CollisionResult  result;    result.clear();

  int ncontacts = collide(&box1, transformB1, &m1, transformM1, request, result);

  if(result.numContacts() > 0) {

    std::vector<fcl::Contact> contacts;
    result.getContacts(contacts);

    // /* Show results in a format suitable for load in matlab */
    // for (unsigned int i=0; i<contacts.size(); ++i)
    //   std::cout << contacts[i].pos[0] << " " << contacts[i].pos[1] << " " << contacts[i].pos[2] << "\n";

    /* Create a vector containing a structure of Points to sort them and eliminate duplicates */
    std::vector<Points> contactPoints;
    eliminateDuplicates(contacts, contactPoints, 0.002); 
    // std::cout << contactPoints.size() << std::endl;

    /* Show results in a format suitable for load in matlab */
    for (unsigned int i=0; i<contactPoints.size(); ++i)
      std::cout << contactPoints[i].x << " " << contactPoints[i].y << " " << contactPoints[i].z << "\n";

    //std::cout << "in collision " << result.numContacts() << " contacts." << std::endl;
    //std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }

  else {
    std::cout << "collision free " << std::endl;
    //std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }

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
