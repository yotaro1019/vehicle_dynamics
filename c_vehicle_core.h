#ifndef _c_vehicle_
#define _c_vehicle_
#include"exchange_data.h"

#ifdef __cplusplus
extern "C" {
#endif


void vehicle_initialize();  //initialize vehicle system

void vehicle_advance( double  cube_fforce[6], Vehicle2Cfd veh2cfd_data );  //advance vehicle system


#ifdef __cplusplus
}
#endif

#endif