#include "planner.h"
#include "car.h"


Planner::Planner(MAP& _map, CAR& _car, vector<CAR>& _obj_cars){

  map = _map;
  car = _car;
  obj_cars = _obj_cars;
}

CAR Planner::planning(){
  CAR tmpCar;
  for (auto it=obj_cars.begin(); it != obj_cars.end(); it++) {
    it->prediction();
  }

  pair<MoveType, int> ego_state = car.getState();

  CAR egoPlan = car;

  vector<pair<MoveType, int>> possible_state;
  if(ego_state.first == KL){
    int currentlane = ego_state.second;
    possible_state.push_back(pair<MoveType, int>(KL,currentlane));
    if (currentlane < map.num_of_lanes-1){
      possible_state.push_back(pair<MoveType, int>(PLC,currentlane+1));
      possible_state.push_back(pair<MoveType, int>(LC,currentlane+1));
    }
    if (currentlane > 1){
      possible_state.push_back(pair<MoveType, int>(PLC,currentlane-1));
      possible_state.push_back(pair<MoveType, int>(LC,currentlane-1));
    }
  } else if (ego_state.first == PLC){
    int currentlane = ego_state.second;
    possible_state.push_back(pair<MoveType, int>(KL,currentlane));
    if (currentlane < map.num_of_lanes-1){
      possible_state.push_back(pair<MoveType, int>(PLC,currentlane+1));
      possible_state.push_back(pair<MoveType, int>(LC,currentlane+1));
    }
    if (currentlane > 1){
      possible_state.push_back(pair<MoveType, int>(PLC,currentlane-1));
      possible_state.push_back(pair<MoveType, int>(LC,currentlane-1));
    }
  } else if (ego_state.first == LC) {
    int targetlane = ego_state.second;
    possible_state.push_back(pair<MoveType, int>(KL,targetlane));
    possible_state.push_back(pair<MoveType, int>(LC,targetlane));
  }

  pair<MoveType, int> best_state;
  double cost = 1e+10;
  double best_t = 0.0;

  for (auto state: possible_state){
    for (double t = 1.0; t <= car.getPredictTime();t+=1.0){
      CAR tmpCar = car;
      tmpCar.stateUpdate(state, obj_cars,t);
    }
  }




  return tmpCar;
}

