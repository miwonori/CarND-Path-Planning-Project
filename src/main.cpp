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
#include <cmath>
#include "map.h"
#include "car.h"
#include "trajectory.h"
#include "planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MAP map;
  
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
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
    map.setWaypoint(x, y, s, d_x, d_y);
  }

  CAR car;
  Trajectory trajectory;
  bool init_flag = true;

  h.onMessage([&car, &trajectory, &map, &init_flag]
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          car.setCar(car_x, car_y, car_s, car_d, deg2rad(car_yaw), car_speed);
          car.setLaneNum(map.lane_width);

          int prev_size = previous_path_x.size();
          double prev_s = 0.0;
          if (prev_size > 1){
            trajectory.setPrevPath(previous_path_x,previous_path_y);
            trajectory.preTrajectory();
            cout << "Car X = " << car_x << ", Car Y = " << car_y 
                << ", Last Prev X = " << previous_path_x[prev_size-1] 
                << ", Last Prev Y = " << previous_path_y[prev_size-1] << endl;
            prev_s = trajectory.getS(map.waypoints_x,map.waypoints_y);           
          } else {
            cout << "Car X = " << car_x << ", Car Y = " << car_y << endl;
          }

          if (init_flag){
            init_flag = false;
            // for (int i=1;i<4;i++){
            //   vector<double> tmp_xy = getXY(car.getS() + (i*0.4), 6.0, 
            //                                 map.waypoints_s, map.waypoints_x,map.waypoints_y);
            //   trajectory.next_x_vals.push_back(tmp_xy[0]);
            //   trajectory.next_y_vals.push_back(tmp_xy[1]);
            // }
          }
          
          cout <<trajectory.next_x_vals.size() << endl;

          /*----- Sensor Fusion Data -----*/
          vector<CAR> obj_cars;
          for (auto it=sensor_fusion.begin();it != sensor_fusion.end();it++){
            CAR obj_car;
            int ID = it.value()[0];
            double X = it.value()[1];
            double Y = it.value()[2];
            double Vx = it.value()[3];
            double Vy = it.value()[4];
            double S = it.value()[5];
            double D = it.value()[6];
            obj_car.setfusiondata(ID, X, Y, Vx, Vy, S, D);
            obj_car.setSpeed();
            obj_car.setYaw();
            obj_car.setLaneNum(map.lane_width);

            obj_cars.push_back(obj_car);
          }

          if (prev_s < trajectory.max_dist){
            Planner planner(map, car, obj_cars);
            CAR next_car = planner.planning();
          }


          /* End of TODO */

          msgJson["next_x"] = trajectory.next_x_vals;
          msgJson["next_y"] = trajectory.next_y_vals;

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