#ifndef _POINT_VEL_ACC_
#define _POINT_VEL_ACC_
#include <iostream>
#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

class Point_vel_acc{

private:
    int npoint = 0;
    bool c_switch = false;

    void read_param(std::string fname);
    
public:
    Point_vel_acc(bool c_switch, std::string fname);
    
};
#endif