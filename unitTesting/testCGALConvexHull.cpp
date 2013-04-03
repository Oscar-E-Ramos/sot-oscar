/*
  2013 Oscar Efrain Ramos Ponce, LAAS-CNRS

  Test for the convex hull of synthetic points
*/


#include <vector>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;

typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Segment_3                              Segment_3;
typedef K::Triangle_3                             Triangle_3;
typedef K::Point_3                                Point_3;
typedef K::Vector_3                               Vector_3;

typedef CGAL::Creator_uniform_3<double, Point_3>  PointCreator;


void generatePointsInPlane(std::vector<Point_3> &P, double min,
			   double max, double delta);
void generatePointsInLine(std::vector<Point_3> &points, double min,
			  double max, double delta);
void rotatePoints(std::vector<Point_3> &points, double angle, char axis);


int main( void )
{
  /* Vector containing the input data points */
  std::vector<Point_3> points;

  double min=-1.0, max=1.0, delta=0.2, angle=0.5;
  char axis='y'; 
  //generatePointsInPlane(points, min, max, delta, angle, axis);
  generatePointsInLine(points, min, max, delta);
  rotatePoints(points, angle, axis);
    
  /* Output to screen the initial points in matlab format */
  std::cout << "Pinit = [ ";
  for(unsigned int i=0; i<points.size(); i++)
    std::cout << points[i] << "; ";
  std::cout << "];" << std::endl;

  /* Define Object to hold the convex hull */
  CGAL::Object ch_object;

  /* Define objects that might be the possible output */
  Polyhedron_3 poly;
  Point_3 point;
  Segment_3 segment;

  /* compute convex hull of non-collinear points */
  CGAL::convex_hull_3(points.begin(), points.end(), ch_object);


  if (assign(point, ch_object)) {
    /* The object is a point */
    std::cout << "The convex hull is a point" << std::endl;
  }

  else if (assign(segment, ch_object)) {
    /* The object is a segment*/
    std::cout << "\nThe convex hull contains 2 points:" << std::endl;
    std::cout << "Pfinal = [ " << segment.point(0) << "; " << segment.point(1)
	      << "];" << std::endl;
  }

  else if (assign(poly, ch_object)) {
    /* The object is a polyhedron*/
    std::cout << "\nThe convex hull contains " << poly.size_of_vertices() << " vertices:" << std::endl;
    /* Output to screen the final points in matlab format */
    std::cout << "Pfinal = [ ";
    for(Polyhedron_3::Vertex_iterator viter = poly.vertices_begin(); viter!=poly.vertices_end(); viter++) 
      std::cout << viter->point() << "; ";
    std::cout << "];" << std::endl;
  }

  return 0;
}



/* Generate the 3d points in a plane*/
void generatePointsInPlane(std::vector<Point_3> &P, double min, 
			   double max, double delta)
{
  double eps = 1e-10;
  
  /* Generate the grid assuming z=0 */
  for (double i=min; i<max+eps; i+=delta){
    for (double j=min; j<max+eps; j+=delta){
      Point_3 ptemp(i,j,0.0);
      P.push_back(ptemp);
    }
  }
}

void rotatePoints(std::vector<Point_3> &P, double angle, char axis)
{

  /* Get the rotation matrix */
  double Rot[9];
  double ca = cos(angle), sa = sin(angle);
  if((axis=='x') || (axis=='X')){
    Rot[0] = 1; Rot[1] = 0;  Rot[2] = 0;
    Rot[3] = 0; Rot[4] = ca; Rot[5] = -sa;
    Rot[6] = 0; Rot[7] = sa; Rot[8] = ca;
  }
  else if((axis=='y') || (axis=='Y')){
    Rot[0] =  ca; Rot[1] = 0; Rot[2] = sa;
    Rot[3] =   0; Rot[4] = 1; Rot[5] = 0;
    Rot[6] = -sa; Rot[7] = 0; Rot[8] = ca;
  }
  else if((axis=='z') || (axis=='Z')){
    Rot[0] = ca; Rot[1] = -sa; Rot[2] = 0;
    Rot[3] = sa; Rot[4] =  ca; Rot[5] = 0;
    Rot[6] =  0; Rot[7] =   0; Rot[8] = 1;
  }
  
  /* Rotate the grid on x,y or z according to Rot */
  double px, py, pz;
  for( unsigned int i=0; i<P.size(); i++){
    // px = Rot[0]*P[i][0] + Rot[1]*P[i][1] + Rot[2]*P[i][2];   // Also possible
    px = Rot[0]*P[i].x() + Rot[1]*P[i].y() + Rot[2]*P[i].z();
    py = Rot[3]*P[i].x() + Rot[4]*P[i].y() + Rot[5]*P[i].z();
    pz = Rot[6]*P[i].x() + Rot[7]*P[i].y() + Rot[8]*P[i].z();
    P[i] = Point_3(px,py,pz);
  }

}

void generatePointsInLine(std::vector<Point_3> &P, double min,
			  double max, double delta)
{
  double eps = 1e-10;
  for (double i=min; i<max+eps; i+=delta){
    Point_3 ptemp(i,1.0,0.0);
    P.push_back(ptemp);
  }
}
