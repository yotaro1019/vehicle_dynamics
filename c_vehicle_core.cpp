#include"c_vehicle_core.h"
#include"vehicle_core.h"
#include<iostream>

Vehicle_model veh;

void vehicle_initialize(){
    veh.vehicle_initialize();
}

void vehicle_advance( Cfd2Vehicle *cfd2veh_data , Vehicle2Cfd *veh2cfd_data){
    veh.vehicle_advance( cfd2veh_data , veh2cfd_data);     //vehicle advance package  
}

void vehicle_initialize_stand_alone(){
    veh.vehicle_initialize_stand_alone();
}

void vehicle_advance_stand_alone(){
    veh.vehicle_advance_stand_alone();
}

