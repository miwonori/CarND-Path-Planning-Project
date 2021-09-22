#include "trajectory.h"

Trajectory::Trajectory() {}

void Trajectory::setPrevPath(vector<double> x, vector<double> y){
  previous_path_x = x;
  previous_path_y = y;
}

void Trajectory::preTrajectory() {
  next_x_vals = previous_path_x;
  next_y_vals = previous_path_y;

  Interval_Dist.clear();
  Interval_Speed.clear();
  Interval_Yaw.clear();
  Interval_accel.clear();
  Interval_jerk.clear();
  
  int NofPrev = next_x_vals.size();
  for (int i=1; i<NofPrev; i++){
    double dist = distance(next_x_vals[i-1],next_y_vals[i-1],next_x_vals[i], next_y_vals[i]);
    Interval_Dist.push_back(dist);
    Interval_Speed.push_back(dist/Interval_Time);
    double tmp_Yaw = getYaw(next_x_vals[i-1],next_y_vals[i-1],next_x_vals[i], next_y_vals[i]);
    Interval_Yaw.push_back(tmp_Yaw);
  }
  for (int i=1; i<NofPrev-1;i++){
    double tmp_accel = (Interval_Speed[i] - Interval_Speed[i-1])/Interval_Time;
    Interval_accel.push_back(tmp_accel);
  }
  for (int i=1; i<NofPrev-2;i++){
    double tmp_jerk = (Interval_accel[i] - Interval_accel[i-1])/Interval_Time;
    Interval_jerk.push_back(tmp_jerk);
  }

}

double Trajectory::getS(const vector<double> &maps_x, const vector<double> &maps_y){
  double S = 0.0;
  if(next_x_vals.size() > 1){
    int last_idx = next_x_vals.size()-1;
    vector<double> SD = getFrenet(next_x_vals[last_idx], next_y_vals[last_idx], Interval_Yaw[last_idx-1],maps_x, maps_y);
    S = SD[0];
  }
  return S;
}


// void Trajectory::makeTrajectory(double car_x, double car_y, double car_yaw, double car_s, int current_lane_N){
//   // x, y points for spline
//   vector<double> points_x;
//   vector<double> points_y;
//   double car_d = lane_width*(current_lane_N-1) + lane_width/2.0;

//   int path_size = previous_path_x.size();
//   if (path_size < 2){
//     double prev_x = car_x - cos(deg2rad(car_yaw));
//     double prev_y = car_y - sin(deg2rad(car_yaw));

//     points_x.push_back(prev_x);
//     points_y.push_back(prev_y);

//     points_x.push_back(car_x);
//     points_y.push_back(car_y);
//   } else {
//     points_x.push_back(previous_path_x[path_size - 2]);
//     points_y.push_back(previous_path_y[path_size - 2]);

//     points_x.push_back(previous_path_x[path_size - 1]);
//     points_y.push_back(previous_path_y[path_size - 1]); 
//   }
//   double Interval = 22 ;
//   for (int i=0; i < 3; i++){
//     vector<double> next_xy = getXY(car_s + (i+1)*Interval, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//     points_x.push_back(next_xy[0]);
//     points_y.push_back(next_xy[1]);
//   }

//   /* Make Spline function */
//   tk::spline path_spline(points_x,points_y);
// }
