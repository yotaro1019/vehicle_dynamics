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
#include "outputlist.h"

#define def_point_nlist 30

using namespace chrono;
using namespace chrono::vehicle;

class Point_vel_acc : public Set_value{

private:
    bool c_switch = false;

    void read_param(std::string fname);
    void initialize_point(WheeledVehicle &veh);

    

    struct Point_data{
        int id;
        std::string name;
        int marker_id;
        ChVector<> pos,rot;
        ChQuaternion<> qrot;
        bool activate = false;
        Point_vel_acc_info pout_file;
    };
     Point_data  point_lists[def_point_nlist];
    
public:
    Point_vel_acc(bool c_switch, std::string fname, WheeledVehicle &veh);
    void record_points_vel_acc(int step, double time, WheeledVehicle &veh);
    void restart(int restart_step);
};

#endif