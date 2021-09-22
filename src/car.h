#ifndef CAR_H
#define CAR_H

#include <vector>
// #include <utility>

using namespace std;

enum MoveType{
  KL,   // keep lane
  PLC,  // prepare lane change
  LC    // lane change
};

class CAR{
  public:

    CAR();
    void setCar(double x, double y, double s, double d, double yaw, double speed);
    void setfusiondata(int id, double x, double y, double vx, double vy, double s, double d);
    void setSpeed();
    void setYaw();
    void setLaneNum(double lane_width);
    void setVxVy();

    double getS();
    pair<MoveType,int> getState();

    void prediction();



  private:

    int ID;
    double X;
    double Y;
    double S;
    double D;
    double Yaw;     // radian
    double Speed;   // m/sec
    double Accel;

    double Vx;
    double Vy;

    int LaneNum;

    pair<MoveType, int> state;      // <type, lane_num>
    double predict_dt = 0.1;    // prediction delta time (sec)
    double predict_T = 6;       // total prediction time (sec)
    vector<pair<double, int>> predicts;  // vector of <S, lane_num>


};


#endif