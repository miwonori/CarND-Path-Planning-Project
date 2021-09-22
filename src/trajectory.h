#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <vector>
#include "spline.h"
#include "helpers.h"

using std::vector;

class Trajectory {
  public:
    Trajectory();
    Trajectory(vector<double> x, vector<double> y);
    
    // virtual ~Trajectory() {}

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    double max_dist = 30.0;

    void setPrevPath(vector<double> x, vector<double> y);
    void preTrajectory();

    double getS(const vector<double> &maps_x, const vector<double> &maps_y);
    // void makeTrajectory(double car_x, double car_y, double car_yaw, double car_s, int current_lane_N);

  private:
    double const Interval_Time = 0.02;
    double const lane_width = 4.0;
    double const Max_speed = MPH2mps(50.0);    
    double const Target_speed = MPH2mps(45.0);
    double const Max_accel = 10.0;
    double const Max_jerk = 10.0;

    vector<double> previous_path_x;
    vector<double> previous_path_y;

    vector<double> Interval_Dist;
    vector<double> Interval_Speed;
    vector<double> Interval_Yaw;
    vector<double> Interval_accel;
    vector<double> Interval_jerk;

};

#endif