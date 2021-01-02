#ifndef _c_vehicle_
#define _c_vehicle_
#include"exchange_data.h"

#ifdef __cplusplus
extern "C" {
#endif


void vehicle_initialize();  //initialize vehicle system(coupling)

void vehicle_advance( Cfd2Vehicle *cfd2veh_data, Vehicle2Cfd *veh2cfd_data );  //advance vehicle system(coupling)
void vehicle_advance_array(double time, double fforce[6], double mesh_vel_acc[6], double obj_vel_rot[9][6]); 

void vehicle_initialize_stand_alone();
void vehicle_advance_stand_alone();


#ifdef __cplusplus
}
#endif

#endif