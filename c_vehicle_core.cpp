#include"c_vehicle_core.h"
#include"vehicle_core.h"

Vehicle_model veh;

void vehicle_initialize(){
    veh.vehicle_initialize();
}

void vehicle_advance( double cube_fforce[6] , Vehicle2Cfd veh2cfd_data){
    veh.vehicle_advance( cube_fforce , veh2cfd_data);     //vehicle advance package   
}


