#include "path.h"
/*==================== Line and Circle Path ==========================*/
// class constructor
PathOval::PathOval(float spacing_val,float xs,float ys, float psis){
  // Initialization of path class
  // track specs
  spec = {{1,0}, //section length and spanned angle, + is ccw
          {M_PI*1.5,M_PI},
          {1,0},
          {M_PI*1.5,M_PI}
         };
  x_s = xs;
  y_s = ys;
  psi_s = psis;
  // setting point and tangent
  // [x end , y end, psir end, cumulative s start, segment length, signed curvature]
  point_n_tangent = std::vector<std::vector<float>> (spec.size(), std::vector<float>(6,0));
  set_point_n_tangent();
  // setting some class variables
  spacing = spacing_val;
  length = point_n_tangent[point_n_tangent.size()-1][3] + point_n_tangent[point_n_tangent.size()-1][4];
  num_points = length/spacing +1;
  spacing = length/num_points;
  // setting path points
  // [x,y,psir, k, s]
  points = std::vector<std::vector<float>> (num_points, std::vector<float>(5,0));
  set_points();
}

std::vector<float> PathOval::get_global(float s,float ey){
  // returns global position from curvilinear coordinate
  // ey>0 in left side of the road
  // retutns (global x, global y, current psir of path, current k, current s)
  std::vector<float> pos(5,0);
  int i = 0, ip = 0;
  float x,y, xf,yf, xs, ys, psir, r, k, ang, direction, index;
  float center_x, center_y, span_ang, angle;
  // wrap the path length
  s = fmod(s,length);
  // Compute the path segment in which the vehicle is evolving
  for(int ii = 0; ii<(int)spec.size();ii++){
	if(s<point_n_tangent[ii][3]+point_n_tangent[ii][4]){
      i = ii;                                         // current section index
      ip = i>0 ? i-1 : (int)point_n_tangent.size()-1; // previous section index
      break;
    }
  }
  // If segment is a straight line
  if (point_n_tangent[i][5] == 0.0){
    // Extract the first final and initial point of the segment
    xf = point_n_tangent[i][0];
    yf = point_n_tangent[i][1];
    xs = point_n_tangent[ip][0];
    ys = point_n_tangent[ip][1];
    psir = point_n_tangent[i][2];
    // Compute the segment length
    float deltaL = point_n_tangent[i][4];
    float reltaL = s - point_n_tangent[i][3];
    // Do the linear combination
    x = (1 - reltaL / deltaL) * xs + reltaL / deltaL * xf + ey * cos(psir + M_PI / 2);
    y = (1 - reltaL / deltaL) * ys + reltaL / deltaL * yf + ey * sin(psir + M_PI / 2);
    k = 0;
    // if segment is circular
  }else{
    k = point_n_tangent[i][5];
    r = 1/point_n_tangent[i][5];    // Extract curvature
    ang = point_n_tangent[i-1][2]; // Extract angle of the tangent at the initial point (i-1)
    // Compute direction of rotation ccw is +
    if (r >= 0){
      direction = 1;
    }else{
      direction = -1;
    }
    // Compute center of the arc
    center_x = point_n_tangent[ip][0]+abs(r)*cos(ang+direction*M_PI/2); // x coordinate center of circle
    center_y = point_n_tangent[ip][1]+abs(r)*sin(ang+direction*M_PI/2); // y coordinate center of circle
    // Compute current span angle
    span_ang = (s-point_n_tangent[i][3])/abs(r);
    psir = wrap(ang + span_ang);                    // Angle of the tangent vector at the current point
    angle = psir - direction*M_PI/2;                // Angle in the current circle
    x = center_x+(abs(r)-direction*ey)*cos(angle);  // x coordinate of the current point
    y = center_y+(abs(r)-direction*ey)*sin(angle);  // y coordinate of the current point
  }
  index = s/spacing;
  pos = {x,y,psir,k,s};
  return pos;
}

std::vector<float> PathOval::get_local(float x,float y,float psir){
  // returns local position of given point in path local frame in the following format
  // [s,e,psir]
  std::vector<float> point;
  return point;
}

void PathOval::set_point_n_tangent(){
  // sets values of point_n_tangent based on specs
  // it sets point_n_tangent for each section of the path as follows
  // [x , y, psir, cumulative s start, segment length, signed curvature]
  // x, y, psir are given at last point of each segment
  // s is given at starting point of each segment
  float l, x ,y, ang, psir, r, center_x, center_y, span_ang, angle;
  int direction;
  std::vector<float> new_line(point_n_tangent[0].size(),0);
  // Do for all segments given in path specs
  for(int i=0; i<(int)spec.size();i++){
    // if current section is straight line
    if (spec[i][1] == 0.0){
      l = spec[i][0];           // Length of the segments
      if(i == 0){
        // if its first section
        ang = psi_s;              // Angle of the tangent vector at the last point of segment
        x = x_s + l * cos(ang); // x coordinate of the last point of the segment
        y = y_s + l * sin(ang); // y coordinate of the last point of the segment
      } else{
        // if it is not first section
        ang = point_n_tangent[i - 1][2];            // Angle of the tangent vector at the starting point of the segment
        x = point_n_tangent[i-1][0] + l * cos(ang); // x coordinate of the last point of the segment
        y = point_n_tangent[i-1][1] + l * sin(ang); // y coordinate of the last point of the segment
      }
      psir = ang;  // Angle of the tangent vector at the last point of the segment
      if (i == 0){
        new_line = {x,y,psir,point_n_tangent[i][3],l,0};
      }else{
        new_line = {x,y,psir,point_n_tangent[i-1][3] + point_n_tangent[i-1][4],l,0};
      }
      point_n_tangent[i] = new_line; // Write the new info
      // if current section is not straight line
    }else{
      l = spec[i][0];                 // Length of the segment
      span_ang = spec[i][1];          // Spanned angle of segment
      r = l/span_ang;                 // Radius of curvature
      // Compute direction of rotation
      if (r >= 0){
        direction = 1;
      }else{
        direction = -1;
      }
      if (i == 0){
        // If it is first segment
        ang = 0;                                                    // Angle of the tangent vector at the
                                                                    // starting point of the segment
        center_x = 0 + abs(r)*cos(ang+direction*M_PI/2);            // x coordinate center of circle
        center_y = 0 + abs(r)*sin(ang+direction*M_PI/2);            // y coordinate center of circle
      }else{
        // If it is not first segment
        ang = point_n_tangent[i - 1][2];                            // Angle of the tangent vector at the
                                                                    // starting point of the segment
        center_x = point_n_tangent[i-1][0]+abs(r)*cos(ang+direction*M_PI/2); // x coordinate center of circle
        center_y = point_n_tangent[i-1][1]+abs(r)*sin(ang+direction*M_PI/2); // y coordinate center of circle
      }

      psir = wrap(ang + span_ang);      // Angle of the tangent vector at the last point of the segment
      angle = psir - direction*M_PI/2;
      x = center_x+abs(r)*cos(angle); // x coordinate of the last point of the segment
      y = center_y+abs(r)*sin(angle); // y coordinate of the last point of the segment

      if (i == 0){
        new_line = {x, y, psir, point_n_tangent[i][3], l, 1/r};
      }else{
        new_line = {x, y, psir, point_n_tangent[i-1][3] + point_n_tangent[i-1][4], l, 1/r};
      }

      point_n_tangent[i] = new_line;  // Write the new info
    }

  }
}

void PathOval::set_points(){
  // produces and sets path points with specified step in the class
  // points = [x, y, psir, k, s]
  float s = 0;
  for(int i = 0; i<num_points;i++){
    points[i] = get_global(s,0.0);
    s += spacing;
  }
}

float PathOval::wrap(float angle){
  // wraps the angle between [0, 2*pi]
  float w_angle;
  w_angle = remainder(angle, 2*M_PI);
  if (w_angle<0){w_angle += 2*M_PI;}
  return w_angle;
}
