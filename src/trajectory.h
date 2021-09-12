#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <vector>
#include "spline.h"
#include "helpers.h"

using std::vector;

class Trajectory
{
  public:

    // Trajectory();
    
    // virtual ~Trajectory() {}

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> previous_path_x;
    vector<double> previous_path_y;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    void makeTrajectory(double car_x, double car_y, double car_yaw, double car_s, int current_lane_N);

  private:
    double const Interval = 0.02;
    double const lane_width = 4.0;
    double const Max_speed = 45;    // MPH

};

#endif