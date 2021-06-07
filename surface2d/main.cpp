#include <cstdio>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <sstream>
#include "gluvi.h"
#include "vec.h"
#include "array2.h"
//#include "coordframe.h"

#include "openglutils.h"
#include "makelevelset2.h"
//#include "marchingsquares.h"
#include "levelset.h"
#include "util.h"
#include "marching_triangles.h"
#include "array2_utils.h"

using namespace std;
int nx,ny;
float dx,lx,ly;
Vec2f origin;
Array2f phi;

std::vector<Vec2f> verts;
std::vector<Vec2ui> edges;

std::vector<Vec2f> gradientVectors;

MarchingTriangles marcher;

void timer(int junk);

float grid_surface(const Vec2f& position) {
   return interpolate_value(position/dx, phi);
   //return interpolate_phi_nc(position, phi,dx);
}


// find distance x0 is from segment x1-x2
static float seg_closest_point(const Vec2f &x0, const Vec2f &x1, const Vec2f &x2, Vec2f& cp)
{
   Vec2f dx(x2-x1);
   double m2=mag2(dx);
   // find parameter value of closest point on segment
   float s12=(float)( dot(x2-x0, dx)/m2 );
   if(s12<0){
      s12=0;
   }else if(s12>1){
      s12=1;
   }
   // and find the distance
   cp = s12*x1+(1-s12)*x2;
   return mag(cp - x0);
}

void process(Vec2f base) {
 
   verts.clear();
   edges.clear();
   

   //read grid dimensions (shouldn't change anyhow)
   nx = 15, ny = 15;
   dx = 1.0 / (double)nx;
   lx = nx*dx;
   ly = ny*dx;
   origin = Vec2f(0,0);
  
   //setup some test geometry
   verts.push_back(base + Vec2f(0.3f,0.4f));
   verts.push_back(base + Vec2f(0.83f,0.81f));
   verts.push_back(base + Vec2f(0.71f,0.1f));
   verts.push_back(base + Vec2f(0.12f,0.12f));
   verts.push_back(base + Vec2f(0.201f,0.75f));

   edges.push_back(Vec2ui(0,1));
   edges.push_back(Vec2ui(1,2));
   edges.push_back(Vec2ui(2,3));
   edges.push_back(Vec2ui(3,4));
   edges.push_back(Vec2ui(4,0));
   
   //compute the signed distance field
   phi.resize(nx+1,ny+1);
   make_level_set2(edges, verts,
                     origin, dx, nx+1, ny+1,
                     phi);
   
   //figure out vectors to the surface for reference
   gradientVectors.resize((nx+1)*(ny+1));
   for(int j = 0; j < ny+1; ++j) {
      for(int i = 0; i < nx+1; ++i) {
         Vec2f start = origin + dx*Vec2f(i,j);
         float closestDist = 1e20;
         Vec2f closestPoint(0,0);
         for(unsigned int p = 0; p < edges.size(); ++p) {
            Vec2f nearPoint;
            float dist_val = seg_closest_point(start, verts[edges[p][0]], verts[edges[p][1]], nearPoint);
            if(dist_val < closestDist) {
               closestDist = dist_val;
               closestPoint = nearPoint;
            }
         }
         gradientVectors[i + j*(nx+1)] = closestPoint;
      }
   }
   
   //run marching triangles
   marcher.dx = dx;
   marcher.origin = Vec2f(0,0);
   marcher.phi = phi;
   marcher.contour_grid();
   
}

void display(void)
{

   
   glClearColor(1,1,1,1);
   glColor3f(0,0,0);
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  
   
   glLineWidth(1);
   draw_grid2d(Vec2f(0,0),dx, (nx-1), (ny-1));
    

   for(int i = 0; i < nx+1; ++i) {
      for(int j = 0; j < ny+1; ++j) {
         if(phi(i,j) < 0)
            glColor3f(0,0,1);   
         else
            glColor3f(1,0,0);
         glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
         Vec2f pos(i*dx,j*dx);
         glLineWidth(3);
         if(i == nx && j == ny) continue;
         
         //draw distance field disks
         //if(fabs(phi(i,j)) < 3*dx)
         draw_circle2d(pos, fabs(phi(i,j)), 1000);
         
         //draw vector to the closest point
         glColor3f(0,0,0);
         //draw_segment2d(pos, gradientVectors[i + j*(nx+1)]);
      }
   }
   
  
  
   glLineWidth(3);
  
   //draw true shape
   glColor3f(0.0f,1.0f,0.0f);
   draw_segmentset2d(verts,edges);

   //draw marching triangles reconstruction
   glColor3f(1,0,0);
   draw_segmentset2d(marcher.x,marcher.edge);

}




void keyPress(unsigned char key, int x, int y) {
  

}

void special_key_handler(int key, int x, int y)
{
   int mods=glutGetModifiers();

}


Vec2f base_pos(0,0);
int main(int argc, char **argv)
{
   Gluvi::init("viewfluid2d", &argc, argv);
   
   glutSpecialFunc(special_key_handler);
   glutKeyboardFunc(keyPress);
   float bottom, left, height;
  
   base_pos = Vec2f(0.3*dx, 0.0*dx);
   process(base_pos);

   nx = 15, ny = 15;
   dx = 1.0 / (double)nx;
   lx = nx*dx;
   ly = ny*dx;
   

   height = max(ly,lx) + dx*4;
   bottom = -0.2;
   left = -0.5;
   Gluvi::PanZoom2D cam(bottom, left, height);
   Gluvi::camera=&cam;

   Gluvi::userDisplayFunc=display;
  
   glutTimerFunc(100, timer, 0);

   glClearColor(0,0,0,1);
  
   Gluvi::run();
   
   return 0;
}



void timer(int junk)
{

   process(base_pos);
	glutPostRedisplay();
	glutTimerFunc(100000, timer, 0);

}

