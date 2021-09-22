#include "map.h"

MAP::MAP(){

}

void MAP::setWaypoint(double x, double y, double s, double d_x, double d_y){
  waypoints_x.push_back(x);
  waypoints_y.push_back(y);
  waypoints_s.push_back(s);
  waypoints_dx.push_back(d_x);
  waypoints_dy.push_back(d_y);
}