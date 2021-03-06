#ifndef _exchange_data_
#define _exchange_data_
#include <iostream>
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include"inp_init_data.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

using namespace chrono;
using namespace chrono::vehicle;

//Components of data structure
struct Components{
    double translation[3] = {0.0};
    double rotation[3] = {0.0};
};

//exchange data structure from vehicle to cfd
struct Vehicle2Cfd{

    //Using translational movement of mesh by non-inertial system
    Components mesh_vel;
    Components mesh_acc;
    Components chassis_vel;
    Components str_vel[4];
    Components wheel_vel[4];
};

//exchange data structure from CFD to vehicle
struct Cfd2Vehicle{
    //Representation in absolute coordinate system
    Components fforce;
    double cfd_time = 0.0;
};


class Exchange_data{
private:
    double direction_axis[3] = {1.0, 1.0, 1.0}; //convert direction of each axis0
    double direction_rot[3] = {1.0, 1.0, 1.0};  //convert direction of each rotation

    void conv_direction(Components &cmp);
    void conv_translation(double data[3]);
    void conv_rotation(double data[3]);


public:
    //Vehicle2Cfd veh2cfd;    //exchange data structure from chrono to cube
    Exchange_data(Input_data &inp);
    void comp_zeros(Components &cmp);
    void comp_set6array(Components &cmp, double array[6]);  
    void comp_get6array(Components &cmp, double array[6]);
    void comp_get3transarray(Components &cmp, double array[3]); 
    void comp_get3rotarray(Components &cmp, double array[3]);  
    void data_unpacking(Cfd2Vehicle *input_data);    
    void data_packing(WheeledVehicle &veh,  Vehicle2Cfd *output_data);
};

#endif