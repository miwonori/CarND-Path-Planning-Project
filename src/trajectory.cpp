#include "trajectory.h"



// Trajectory::Trajectory() { }

void Trajectory::makeTrajectory(double car_x, double car_y, double car_yaw, double car_s, int current_lane_N){
  // x, y points for spline
  vector<double> points_x;
  vector<double> points_y;
  double car_d = lane_width*(current_lane_N-1) + lane_width/2.0;

  int path_size = previous_path_x.size();
  if (path_size < 2){
    double prev_x = car_x - cos(deg2rad(car_yaw));
    double prev_y = car_y - sin(deg2rad(car_yaw));

    points_x.push_back(prev_x);
    points_y.push_back(prev_y);

    points_x.push_back(car_x);
    points_y.push_back(car_y);
  } else {
    points_x.push_back(previous_path_x[path_size - 2]);
    points_y.push_back(previous_path_y[path_size - 2]);

    points_x.push_back(previous_path_x[path_size - 1]);
    points_y.push_back(previous_path_y[path_size - 1]); 
  }
  double Interval = 22 ;
  for (int i=0; i < 3; i++){
    vector<double> next_xy = getXY(car_s + (i+1)*Interval, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    points_x.push_back(next_xy[0]);
    points_y.push_back(next_xy[1]);
  }

  /* Make Spline function */
  tk::spline path_spline(points_x,points_y);
}
