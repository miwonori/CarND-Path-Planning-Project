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
  double oldSpeed;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &oldTime, &newTime, &delTime, &first_flag,
               &oldSpeed]
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
          
          if (first_flag) {
            first_flag = 0;
            oldSpeed = car_speed;
            newTime = std::chrono::system_clock::now();
          }

          /* Check Main Loop Time Interval */
          oldTime = newTime;
          newTime = std::chrono::system_clock::now();
          delTime = newTime-oldTime;
          // cout << "Time Interval = " << delTime.count() << endl;

          /* Current Position in Frenet Coordinate */
          // cout << "Car Pos X = " << car_x << ", Car Pos Y = " << car_y << ", Car Pos Yaw = " << car_yaw << endl;
          // cout << "Car S = " << car_s << ", Car D = "<< car_d << endl;
          int current_lane_N = ceil(car_d/4);
          // cout << "Current Lane Number =" << current_lane_N << endl;

          int WP_N ;
          WP_N = NextWaypoint(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
          // cout << "WayPoint N = " << WP_N << ", Next WayPoint S =" << map_waypoints_s[WP_N] << endl;

          /* if next way point is closed increas waypoint number */
          double dist_to_WP = map_waypoints_s[WP_N] - car_s ;
          if (dist_to_WP < 5.0){WP_N += 1;}
          // cout << "Distance del S : " << map_waypoints_s[WP_N] - car_s << endl;

          int path_size = previous_path_x.size();
          std::cout << "previous Path_size = " << path_size << std::endl;
          
          /* Find Spline Path */
          vector<double> X;
          vector<double> Y;

          int num_of_mid_point = 3;
          double next_WP_s = map_waypoints_s[WP_N];
          double del_s = (next_WP_s - car_s)/(num_of_mid_point + 1);
          double mid_s;
          double mid_d;
          vector<double> mid_XY;
          vector<double> mid_SD;
          
          X.push_back(car_x);
          Y.push_back(car_y);
          mid_d = lane_width/2 + (lane_width * (current_lane_N-1));
          if (path_size != 0){
            for (int i = 0; i < path_size; i++){
              mid_SD = getFrenet(previous_path_x[i], previous_path_y[i], deg2rad(car_yaw), 
                                 map_waypoints_x, map_waypoints_y);
              if (mid_SD[0] < next_WP_s){
                X.push_back(previous_path_x[i]);
                Y.push_back(previous_path_y[i]);
              } else {
                break;
              }
            }
          } else {
            for (int i = 0; i < num_of_mid_point; i++){
              mid_s = car_s + (del_s * (i+1)) ;
              mid_XY = getXY(mid_s, mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              X.push_back(mid_XY[0]);
              Y.push_back(mid_XY[1]);
            }
          }

          mid_XY = getXY(map_waypoints_s[WP_N], mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          X.push_back(mid_XY[0]);
          Y.push_back(mid_XY[1]);
          // cout << "mid_XY X = " << mid_XY[0] << ", mid_XY Y = " << mid_XY[1] << endl;
          cout << "1" << endl;
          tk::spline path_spline(X,Y);
           cout << "2" << endl;
          double spline_Y;

          double Max_Speed = 45;    //MPH
          double max_acc = 9;
          double max_jerk_acc = 9 ; //m/s^3
          double del_t = 0.02;
          double dist_inc = 0.4;
          // double acc = MPH2mps(car_speed - oldSpeed)/del_t;
          // double acc = MPH2mps(car_speed - oldSpeed)/delTime.count();
          // acc += max_jerk_acc*del_t;
          // acc += max_jerk_acc*delTime.count();
          // if (acc > max_acc){
          //   acc = max_acc;
          // }
          // cout << "old spd = " << oldSpeed << ", new spd = " << car_speed << endl;
          // cout << "Acceleration = " << acc << endl;

          // // dist_inc = 0.4;
          // // dist_inc = ((MPH2mps(car_speed)*delTime.count()) + (0.5*max_acc*delTime.count()));
          // if (car_speed < Max_Speed){
          //   // dist_inc = ((MPH2mps(car_speed)*del_t) + (0.5*acc*del_t*del_t));
          //   dist_inc = ((MPH2mps(car_speed)*delTime.count()) + (0.5*max_acc*delTime.count()*delTime.count()));
          // } else {
          //   dist_inc = (MPH2mps(Max_Speed)*del_t);
          // }
          
          // if (dist_inc > 0.4){
          //   dist_inc = 0.4;
          // }

          cout << "Est dist increment :" << dist_inc << endl;
          for (int i = 0; i < 20; i++){
            mid_s = car_s + (dist_inc*(i+1)) ;
            mid_XY = getXY(mid_s, mid_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_Y = path_spline(mid_XY[0]);
            // if (i == 0){
            //   cout << "First WayPoint X = " << mid_XY[0] << ", First WayPoint Y = " << spline_Y << endl;
            // }
            // cout << " Next X = " << mid_XY[0] << ", Next Y = " << spline_Y << endl;
            next_x_vals.push_back(mid_XY[0]);
            next_y_vals.push_back(spline_Y);
          }
           cout << "4" << endl;
          // cout << "Last WayPoint X = " << mid_XY[0] << ", Last WayPoint Y = " << spline_Y << endl;

          
          // cout << "Close X = " << map_waypoints_x[WP_N] << ", Close Y = " << map_waypoints_y[WP_N] << endl;

          // for (int i = WP_N; i < WP_N + 50; ++i){
          //   next_x_vals.push_back(map_waypoints_x[i]);
          //   next_y_vals.push_back(map_waypoints_y[i]);
          // }

          // double pos_x;
          // double pos_y;
          // double angle;
          // int path_size = previous_path_x.size();
          // std::cout << "previous Path_size = " << path_size << std::endl;

          // for (int i = 0; i < path_size; ++i){
          //   next_x_vals.push_back(previous_path_x[path_size -1]);
          //   next_y_vals.push_back(previous_path_y[path_size -1]);
          // }

          // if (path_size == 0){
          //   pos_x = car_x;
          //   pos_y = car_y;
          //   angle = deg2rad(car_yaw);
          // } else {
          //   pos_x = previous_path_x[path_size -1];
          //   pos_y = previous_path_y[path_size -1];
          //   double pos_x2 = previous_path_x[path_size -2];
          //   double pos_y2 = previous_path_y[path_size -2];
          //   angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
          // }
          
          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; ++i){
          //   next_x_vals.push_back(pos_x + (dist_inc)*cos(angle +(i+1)*(pi()/100)));
          //   next_y_vals.push_back(pos_y + (dist_inc)*sin(angle +(i+1)*(pi()/100)));
          //   pos_x += (dist_inc)*cos(angle +(i+1)*(pi()/100));
          //   pos_y += (dist_inc)*sin(angle +(i+1)*(pi()/100));
          // }

          oldSpeed = car_speed; 
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