#include "car.h"
#include <math.h>

CAR::CAR(){

}

void CAR::setCar(double x, double y, double s, double d, double yaw, double speed){
  X = x;
  Y = y;
  S = s;
  D = d;
  Yaw = yaw;
  Speed = speed;
}

void CAR::setfusiondata(int id, double x, double y, double vx,
                        double vy, double s, double d){
  ID = id;
  X = x;
  Y = y;
  Vx = vx;
  Vy = vy;
  S = s;
  D = d;
  setLaneNum(4.0);
  state = pair<MoveType, int>(KL, LaneNum);
}

void CAR::setSpeed(){
  Speed = sqrt(Vx*Vx + Vy*Vy);
}

void CAR::setYaw(){
  Yaw = atan2(Vy, Vx);
}

void CAR::setLaneNum(double lane_width){
  LaneNum = ceil(D/lane_width);
}

void CAR::setVxVy(){
    
}

void CAR::stateUpdate(pair<MoveType, int>& _state, vector<CAR>& obj_cars, double t){
  state = _state;
  MoveType mov = state.first;
  int lane = state.second;
  switch (mov) {
    case KL:
      // Accel = estMaxAccel(obj_cars, t);
      break;
    case PLC:
      break;
    case LC:
      break;
  }

}

double CAR::getS(){
  return S;
}

pair<MoveType,int> CAR::getState(){
  return state;
}

double CAR::getPredictTime(){
  return predict_T;
}

void CAR::prediction(){
  predicts.clear();
  MoveType move = state.first;
  int target_lane = state.second;

  switch (move){
    case KL:
      for (double i=0; i<predict_T; i += predict_dt){
        double tmp_S = S + (Speed*i) + (0.5*Accel*i*i);
        predicts.push_back({tmp_S,LaneNum});
      }
      break;
    case PLC:
      for (double i=0; i<predict_T; i += predict_dt){
        double tmp_S = S + (Speed*i) + (0.5*Accel*i*i);
        predicts.push_back({tmp_S,LaneNum});
      }
      break;
    case LC:
      for (double i=0; i<predict_T; i += predict_dt){
        double tmp_S = S + (Speed*i) + (0.5*Accel*i*i);
        predicts.push_back({tmp_S,target_lane});
      }
      break;
  }

}

