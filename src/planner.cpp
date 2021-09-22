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

  piar<MoveType, int> ego_stat = car.



  return tmpCar;
}

