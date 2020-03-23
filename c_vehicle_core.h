#ifndef _c_vehicle_
#define _c_vehicle_


#ifdef __cplusplus
extern "C" {
#endif


void vehicle_initialize();  //initialize vehicle system

void vehicle_advance( double cube_fforce[6] );  //advance vehicle system


#ifdef __cplusplus
}
#endif

#endif