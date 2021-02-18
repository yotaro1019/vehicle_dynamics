#ifndef _POINT_VEL_ACC_
#define _POINT_VEL_ACC_
#include <iostream>
#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "set_value.h"

using namespace chrono;
using namespace chrono::vehicle;

class Point_vel_acc : public Set_value{

private:
    bool c_switch = false;

    void read_param(std::string fname);
    void initialize_point(WheeledVehicle &veh);
    void log_points_vel_acc(WheeledVehicle &veh);
    

    struct Point_data{
        int id;
        std::string name;
        ChVector<> pos,rot;
        ChQuaternion<> qrot;
    };
     std::vector<Point_data>  point_lists;
    
public:
    Point_vel_acc(bool c_switch, std::string fname, WheeledVehicle &veh);
    
};

#endif