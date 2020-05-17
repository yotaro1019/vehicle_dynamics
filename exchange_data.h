#ifndef _exchange_data_
#define _exchange_data_
#include <iostream>
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include"inp_init_data.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

using namespace chrono;
using namespace chrono::vehicle;
//exchange data structure from vehicle to cfd
struct Vehicle2Cfd{

    //Using translational movement of mesh by non-inertial system
    double mesh_vel[3] = {0.0};
    double mesh_acc[3] = {0.0};
    
};

//exchange data structure from CFD to vehicle
struct Cfd2Vehicle{
    //Representation in absolute coordinate system
    double chassis_fforce[3]; // = {0.0, 0.0, 0.0};
    double chassis_fmoment[3] = {0.0, 0.0, 0.0};
};

class Exchange_data{
private:
    double direction_axis[3] = {1.0, 1.0, 1.0}; //convert direction of each axis0
    double direction_rot[3] = {1.0, 1.0, 1.0};  //convert direction of each rotation
    double wheel_angvel[20]; // angular velocity of each wheels

    void conv_dir(double data[3]);
    void conv_rot(double data[3]);


public:
    Vehicle2Cfd veh2cfd;    //exchange data structure from chrono to cube
    Exchange_data(Input_data &inp);
    void data_unpacking(Cfd2Vehicle *input_data);    
    void data_packing(WheeledVehicle &veh,  Vehicle2Cfd *output_data);
};

#endif