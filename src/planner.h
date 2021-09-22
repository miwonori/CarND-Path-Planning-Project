#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "map.h"
#include "car.h"

class Planner {

  public:
    Planner(MAP& _map, CAR& _car, vector<CAR>& _obj_cars);
    
    CAR planning();

  private:
    MAP map;
    CAR car;
    vector<CAR> obj_cars;   

};


#endif
