#ifndef MAP_H
#define MAP_H

#include <vector>

using namespace std;

class MAP{
  public:

    MAP();
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    double max_s = 6945.554;
    double lane_width = 4.0;
    int num_of_lanes = 3;

    void setWaypoint(double x, double y, double s, double d_x, double d_y);

  private:

};


#endif