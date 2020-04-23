#ifndef _exchange_data_
#define _exchange_data_
#include <iostream>
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include"inp_init_data.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

using namespace chrono;
using namespace chrono::vehicle;
//exchange data structure from chrono to cube
struct Vehicle2Cfd{
    double mesh_vel[3];
    double mesh_acc[3];
};

class Exchange_data{
private:
    double direction_axis[3] = {0}; //convert direction of each axis
    double direction_rot[3] = {0};  //convert direction of each rotation

    void conv_dir(double data[3]);
    void conv_rot(double data[3]);


public:
    Vehicle2Cfd veh2cfd;    //exchange data structure from chrono to cube
    Exchange_data(Input_data &inp);
    void data_packing(WheeledVehicle &veh);
};

#endif