#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <chrono>
#include "spline.h"
#include <cmath>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  int index = 0;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // std::chrono::milliseconds oldTime;
  // std::chrono::milliseconds newTime;
  std::chrono::system_clock::time_point oldTime;
  std::chrono::system_clock::time_point newTime;
  std::chrono::duration<double> delTime;
  bool first_flag = true;
  double car_speed_old;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &oldTime, &newTime, &delTime, &first_flag,
               &car_speed_old, &index]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];         // deg
          double car_speed = j[1]["speed"];     // MPH

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double lane_width = 4.0;
          double Max_speed = 45;    // MPH
          
          if (first_flag) {
            first_flag = 0;
            car_speed_old = car_speed;
            newTime = std::chrono::system_clock::now();
          }

          /* Check Main Loop Time Interval */
          oldTime = newTime;
          newTime = std::chrono::system_clock::now();
          delTime = newTime-oldTime;
          // cout << "Time Interval = " << delTime.count() << endl;

          /* Current Position in Frenet Coordinate */
          // cout << "Car Pos X = " << car_x << ", Car Pos Y = " << car_y << ", Car Pos Yaw = " << car_yaw << "Car S = " << car_s << ", Car D = "<< car_d << endl;
          if (car_speed >= Max_speed){
            cout << "        Car Speed = " << car_speed << endl;
          }
          
          int current_lane_N = ceil(car_d/4.0);
          // cout << "Current Lane Number =" << current_lane_N << endl;

          int WP_N ;
          WP_N = NextWaypoint(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
          // cout << "Car S = " << car_s << ", WayPoint N = " << WP_N << ", Next WayPoint S =" << map_waypoints_s[WP_N] << endl;

          /* if next way point is closed increas waypoint number */
          double dist_to_WP = map_waypoints_s[WP_N] - car_s ;
          if (dist_to_WP < 2.0){WP_N += 1;}
          // cout << "Distance del S : " << map_waypoints_s[WP_N] - car_s << endl;

          int path_size = previous_path_x.size();
          // std::cout << "previous Path_size = " << path_size << std::endl;
          
          /* Find Spline Path */
          vector<double> X;
          vector<double> Y;

          // int num_of_mid_point = 3;
          // double next_WP_s = map_waypoints_s[WP_N];
          // double del_s = (next_WP_s - car_s)/(num_of_mid_point + 1);
          double mid_s = car_s;
          double mid_d;
          vector<double> mid_XY;
          vector<double> mid_SD;
          int use_size = 4 ;
          
          X.push_back(car_x);
          Y.push_back(car_y);

          mid_d = lane_width/2 + (lane_width * (current_lane_N-1));

          if (path_size != 0){
            for (int i = 0; i < use_size; i++){
              mid_SD = getFrenet(previous_path_x[i], previous_path_y[i], deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
              if (mid_SD[0] > mid_s){
                X.push_back(previous_path_x[i]);
                Y.push_back(previous_path_y[i]);
                mid_s = mid_SD[0];
              }
            }
          }

          mid_XY = getXY(map_waypoints_s[WP_N], mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          X.push_back(mid_XY[0]);
          Y.push_back(mid_XY[1]);

          mid_XY = getXY(map_waypoints_s[WP_N+1], mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          X.push_back(mid_XY[0]);
          Y.push_back(mid_XY[1]);

          // for (int i = 0; i <X.size();i++){
          //   cout << "X = " << X[i] << "; Y = " << Y[i] << endl;
          // }

          /* Make Spline function */
          tk::spline path_spline(X,Y);
          double spline_Y;
          double dist_inc = 0.4;

          use_size = 0;
          mid_s = car_s;
          if ((path_size != 0)&&(use_size !=0)){
            for (int i = 0; i < use_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
              mid_SD = getFrenet(previous_path_x[i], previous_path_y[i], deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
              mid_s = mid_SD[0]; 
            }
          }

          double del_T =  delTime.count() / (50 - path_size);
          if ((del_T < 0.0001) || isfinite(del_T)){
            del_T = 0.02;
          }

          double speed = MPH2mps(car_speed);
          for (int i = use_size; i < 50; i++){
            
            if(speed < MPH2mps(Max_speed)){
              speed = speed + (10 * del_T) ;
            }

            if (speed > MPH2mps(Max_speed)){
              speed = MPH2mps(Max_speed);
            }
            dist_inc = speed * del_T;
            cout << "WayPoint = " << WP_N <<  ", Set Speed = " << speed << ", Set dist = " << dist_inc << ", Del T = " << del_T << endl;

            mid_s = mid_s + dist_inc ;
            mid_XY = getXY(mid_s, mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_Y = path_spline(mid_XY[0]);
            next_x_vals.push_back(mid_XY[0]);
            next_y_vals.push_back(spline_Y);
          }
          cout << "previous Path_size = " << path_size << " Next Vals Size = " << next_x_vals.size() << endl;
          // double acc = (car_speed - car_speed_old) / delTime.count() ;
          // cout << "index: " << index << ", del Vel = " << (car_speed - car_speed_old) << ", delta T = " << delTime.count() << ", Accel = " << acc <<  endl;
          // cout << "Used Path = " << (50 - path_size) << ", Used Time = " << delTime.count()*1000 << endl;
          // cout << "Delta T by used Samples = " << delTime.count() / (50 - path_size) << endl;

          car_speed_old = car_speed; 
          index++ ;
          /* End of TODO */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1",port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}