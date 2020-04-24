#include"c_vehicle_core.h"
#include"vehicle_core.h"
#include<iostream>

Vehicle_model veh;

void vehicle_initialize(){
    veh.vehicle_initialize();
}

void vehicle_advance( double cube_fforce[6] , Vehicle2Cfd *veh2cfd_data){
    std::cout << "before_call\tc_vehicle\n" << veh2cfd_data->mesh_vel[0] <<  " " << veh2cfd_data->mesh_vel[1] <<  " " << veh2cfd_data->mesh_vel[2] <<  "\n";
    veh.vehicle_advance( cube_fforce , *veh2cfd_data);     //vehicle advance package  
    std::cout << "after_call\tc_vehicle\n" << veh2cfd_data->mesh_vel[0] <<  " " << veh2cfd_data->mesh_vel[1] <<  " " << veh2cfd_data->mesh_vel[2] <<  "\n";
}


