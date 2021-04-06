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

void vehicle_advance_array(double time, double fforce[6], double mesh_vel_acc[6], double obj_vel_rot[9][6]){
    Cfd2Vehicle c2v;
    Vehicle2Cfd v2c;

    for(int i = 0; i<3; i++){
       c2v.fforce.translation[i] = fforce[i];
       c2v.fforce.rotation[i]    = fforce[i+3];
    }
    
    veh.vehicle_advance(&c2v, &v2c);

    //mesh_vel_acc
    for(int i = 0; i<3; i++){
       mesh_vel_acc[i]   = v2c.mesh_vel.translation[i];
       mesh_vel_acc[i+3] = v2c.mesh_acc.translation[i];
    }

    //obj_vel_rot
    for(int obj_id = 0; obj_id<9; obj_id++){

        switch(obj_id){
            case 0:   //chassis
                for(int i = 0; i<3; i++){
                    obj_vel_rot[obj_id][i]   = v2c.chassis_vel.translation[i];
                    obj_vel_rot[obj_id][i+3] = v2c.chassis_vel.rotation[i];
                }
                break;
            
            case 1:   //str_FL
            case 2:   //str_FR
            case 3:   //str_RL
            case 4:   //str_RR
                for(int i = 0; i<3; i++){
                    obj_vel_rot[obj_id][i]   = v2c.str_vel[obj_id-1].translation[i];
                    obj_vel_rot[obj_id][i+3] = v2c.str_vel[obj_id-1].rotation[i];
                }
                break;  

            case 5:   //wheel_FL
            case 6:   //wheel_FR
            case 7:   //wheel_RL
            case 8:   //wheel_RR
                for(int i = 0; i<3; i++){
                    obj_vel_rot[obj_id][i]   = v2c.wheel_vel[obj_id-5].translation[i];
                    obj_vel_rot[obj_id][i+3] = v2c.wheel_vel[obj_id-5].rotation[i];
                }
                break;  
        }
    }
}


void vehicle_initialize_stand_alone(int &begin_step, int &end_step){
    veh.vehicle_initialize_stand_alone(begin_step, end_step);
}

void vehicle_advance_stand_alone(){
    veh.vehicle_advance_stand_alone();
}

