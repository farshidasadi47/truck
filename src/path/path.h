#ifndef PATH_H
#define PATH_H
#include <math.h>
#include <vector>
/*==================== Line and Circle Path ==========================*/
class PathOval{//generates path points, heading angle, lateral error, etc
  public:
    // class variables and initialization
    float spacing, length;
    int num_points;
    float x_s, y_s, psi_s; // start of the path
    // spec [length, angle spanned]
    std::vector<std::vector<float>> spec;
    // point_n_tangent [x, y, psir, cumulative s start, segment length, signed curvature], x,y,psir are at last point, s is in start point
    std::vector<std::vector<float>> point_n_tangent;
    std::vector<std::vector<float>> points;
    // class constructor
    PathOval(float spacing_val, float xs, float ys, float psis);
    // public functions
    std::vector<float> get_global(float s,float ey);
    std::vector<float> get_local(float x,float y,float psir);
  //private:
    void set_point_n_tangent();
    void set_points();
    float wrap(float angle);
};
#endif // PATH_H
/* PathOval implementation
#include "path.h"
int main()
{
   PathOval path(0.01);
   std::cout << path.length << std::endl;
   std::cout << path.num_points << std::endl;
   std::cout << path.spec.size() << std::endl;
   for(int i = 0; i<(int)path.point_n_tangent.size();i++){
       for(int j = 0;j<6;j++){
           printf("%+12.4f",path.point_n_tangent[i][j]);
       }
       std::cout << std::endl;
   }

   std::vector<float> vec;
   std::vector<std::vector<float>> points;
   vec = path.get_global(9.89048622548,0.0);
   printf("%+12.4f, %+12.4f, %+12.4f, %+12.4f\n\n\n",vec[0],vec[1],vec[2],vec[3]);
   points = path.points;
   for(int i = 0;i<100;i++){
       vec = points[i];
       printf("%+12.4f, %+12.4f, %+12.4f, %+12.4f\n",vec[0],vec[1],vec[2],vec[3]);
   }
   return 0;
}
*/
