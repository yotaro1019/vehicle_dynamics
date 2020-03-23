#include"c_vehicle.h"
#include"vehicle_core.h"

Chrono_Vehicle_Model vehicle_model;

void vehicle_initialize(){
    vehicle_model.vehicle_initialize();
}

void vehicle_advance( double cube_fforce[6] ){
    vehicle_model.vehicle_advance(*outside_time, *outside_step, cube_fforce, mesh_vel_acc, chassis_vel_rot, str_angle_array, wheel_rot_array);     //vehicle advance package   
}


